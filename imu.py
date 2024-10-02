import math

FK = 0.1

def get_angle_x(angle_x, acc, gyro, last_ts_gyro, ts):
    angle_ax = 90 + math.degrees(math.atan2(acc.y, acc.z))
    if angle_ax > 180:
        angle_ax -= 360

    if angle_x is None:
        angle_x = angle_ax
    else:
        angle_x = (1 - FK) * (angle_x + math.degrees(gyro.x * (ts - last_ts_gyro) / 1000)) + FK * angle_ax
    last_ts_gyro = ts
    return angle_x, last_ts_gyro