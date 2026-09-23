#!/usr/bin/env python3
"""Offline scoring replay of a sqlite ROS bag with recorded confidence hypotheses.

Requires numpy only. Reuses recorded timestamps/hypotheses to isolate classifier
changes; this does not test the live node's timing, DDS, or TF subscriptions.
Supports uncompressed little-endian CDR LaserScan, String, and TFMessage.
"""
import argparse
import collections
import json
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from armatron.scan_evidence import ScanEvidence, EvidenceDwell, rotation


class CDR:
    def __init__(self, blob):
        if blob[:2] != b'\x00\x01':
            raise ValueError('Only little-endian CDR is supported')
        self.blob, self.offset = blob, 4

    def read(self, fmt, alignment):
        self.offset += (-(self.offset-4)) % alignment
        values = struct.unpack_from('<'+fmt, self.blob, self.offset)
        self.offset += struct.calcsize('<'+fmt)
        return values

    def string(self):
        size = self.read('I', 4)[0]
        value = self.blob[self.offset:self.offset+size-1].decode()
        self.offset += size
        return value

    def header(self):
        sec, ns = self.read('iI', 4)
        return sec+ns*1e-9, self.string()


def replay(path, output):
    scans, records, transforms = {}, [], {}
    with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
        topics = dict(db.execute('select id,name from topics'))
        start = db.execute('select min(timestamp) from messages').fetchone()[0]
        for topic_id, timestamp, blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
            topic = topics[topic_id]
            if topic not in ('/scan', '/tf_static', '/lidar/confidence'):
                continue
            reader = CDR(blob)
            if topic == '/lidar/confidence':
                records.append(((timestamp-start)*1e-9, json.loads(reader.string())))
            elif topic == '/tf_static':
                for _ in range(reader.read('I', 4)[0]):
                    _, parent = reader.header()
                    child = reader.string()
                    value = (parent, reader.read('7d', 8))
                    if child in transforms and transforms[child] != value:
                        raise ValueError('Changing static TF is unsupported')
                    transforms[child] = value
            else:
                stamp, frame = reader.header()
                angle_min, _, increment, _, _, low, high = reader.read('7f', 4)
                size = reader.read('I', 4)[0]
                ranges = np.array(reader.read(str(size)+'f', 4))
                ranges[(ranges < low) | (ranges > high) | ~np.isfinite(ranges)] = np.nan
                angles = angle_min+np.arange(size)*increment
                scans[stamp] = (frame, np.column_stack((ranges*np.cos(angles), ranges*np.sin(angles))))
    for stamp, (frame, points) in list(scans.items()):
        parent, values = transforms[frame]
        if parent != 'base_link':
            raise ValueError('Replay requires direct base_link -> scan static TF')
        x, y, _, qx, qy, qz, qw = values
        if abs(qx) > 0.01 or abs(qy) > 0.01:
            raise ValueError('Nonplanar mounting is unsupported')
        yaw = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        scans[stamp] = points @ rotation(yaw).T + [x, y]
    if not scans or not records:
        raise ValueError('Bag must contain scans and recorded confidence hypotheses')
    times = np.array(sorted(scans))
    engine, dwell = ScanEvidence(), EvidenceDwell()
    counts, skipped = collections.Counter(), 0
    with output.open('x', encoding='utf-8') as file:
        for t, old in records:
            if 'hypotheses' not in old:
                skipped += 1
                continue
            stamps = [old['reference_stamp'], old['stamp']]
            closest = [float(times[np.argmin(abs(times-s))]) for s in stamps]
            if any(abs(a-b) > 1e-5 for a, b in zip(stamps, closest)):
                skipped += 1
                continue
            result = engine.analyze(scans[closest[0]], scans[closest[1]], old['hypotheses'])
            if old.get('solver', {}).get('valid') != '1':
                result.update(state='TRACKING_UNRELIABLE', reason='native solver invalid or absent')
            result.update(time=t, confirmed=dwell.update(stamps[1], result['state']))
            counts[result['confirmed']] += 1
            file.write(json.dumps(result, allow_nan=False)+'\n')
    return {'confirmed_counts': dict(counts), 'skipped_missing_inputs': skipped}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('db3', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(replay(args.db3, args.output), indent=2))
