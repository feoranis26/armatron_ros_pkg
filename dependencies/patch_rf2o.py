#!/usr/bin/env python3
"""Apply the recorded diagnostics patch without overwriting local RF2O edits."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkout', type=Path, help='external rf2o_laser_odometry checkout')
    args = parser.parse_args()
    repo = args.checkout.resolve(strict=True)
    patch = Path(__file__).with_name('rf2o-confidence.patch').resolve()
    def git(*argv):
        return subprocess.run(['git', '-C', str(repo), *argv], capture_output=True, text=True)
    top = git('rev-parse', '--show-toplevel')
    if top.returncode or Path(top.stdout.strip()).resolve() != repo:
        parser.error('checkout must be the root of the external RF2O git repository')
    # Validate metadata before touching source; installations with the previous
    # Humble dependency correction are supported as well as pristine upstream.
    manifest = repo/'package.xml'
    tree = ET.parse(manifest)
    if tree.getroot().findtext('name') != 'rf2o_laser_odometry':
        parser.error('checkout is not rf2o_laser_odometry')
    ready = git('apply', '--check', str(patch))
    if ready.returncode == 0:
        applied = git('apply', str(patch))
        if applied.returncode:
            raise SystemExit(applied.stderr)
        print('Applied RF2O confidence diagnostics patch')
    elif git('apply', '--reverse', '--check', str(patch)).returncode == 0:
        print('RF2O confidence diagnostics patch already applied')
    else:
        raise SystemExit('RF2O source differs from the supported revision; no changes made.\n'+ready.stderr)
    root = tree.getroot()
    for tag in ('build_depend', 'run_depend'):
        for element in list(root.findall(tag)):
            if element.text == 'cmake_modules':
                root.remove(element)
        for dep in ('nav_msgs', 'diagnostic_msgs'):
            if not any(e.text == dep for e in root.findall(tag)):
                ET.SubElement(root, tag).text = dep
    tree.write(manifest, encoding='utf-8', xml_declaration=True)
    print('Humble metadata checked. Rebuild rf2o_laser_odometry, then refresh armatron on x86.')


if __name__ == '__main__':
    main()
