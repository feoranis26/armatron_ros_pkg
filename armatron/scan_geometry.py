"""Mask self-return endpoints, leaving neighbouring rays and near obstacles intact."""
import math


def validate_boxes(boxes):
    for box in boxes:
        if (len(box) != 5 or not all(math.isfinite(v) for v in box) or
                box[2] <= 0 or box[3] <= 0):
            raise ValueError('Each box must be [center_x, center_y, width, height, yaw_radians]')
    return boxes


def mask_ranges(ranges, angle_min, angle_increment, range_min, range_max,
                boxes, padding=0., scan_to_mask=(0., 0., 0.)):
    tx, ty, yaw = scan_to_mask
    c, s = math.cos(yaw), math.sin(yaw)
    output = list(ranges)
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index*angle_increment
        sx, sy = distance*math.cos(angle), distance*math.sin(angle)
        x, y = c*sx-s*sy+tx, s*sx+c*sy+ty
        for cx, cy, width, height, heading in boxes:
            bc, bs = math.cos(heading), math.sin(heading)
            dx, dy = x-cx, y-cy
            if (abs(bc*dx+bs*dy) <= width/2+padding and
                    abs(-bs*dx+bc*dy) <= height/2+padding):
                # Invalid observation, not evidence of free space behind the post.
                output[index] = float('nan')
                break
    return output
