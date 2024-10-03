import pyrealsense2 as rs
import numpy as np
import cv2

from imu import get_angle_x

pipeline = rs.pipeline()
config = rs.config()

# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))

#configure stream depth
W=640
H=480

RECT_PERCENT = 4
PERCENT_DIF = 3
HALF_FOV = 34

# def get_dist(depth_frame, perc, image):
#     if perc > 13:
#         return "Zero", image
#     count = 0
#     temp_sum = 0
#     min_dist = 120
#     sq_x0 = W // 2 - int(W * perc / 200)
#     sq_x1 = W // 2 + int(W * perc / 200)
#     sq_y0 = H // 2 - int(H * perc / 200)
#     sq_y1 = H // 2 + int(H * perc / 200)
#     for i in range(sq_x0, sq_x1):
#         for j in range(sq_y0, sq_y1):
#             dist = depth_frame.get_distance(i, j)
#             if dist:
#                 temp_sum += dist
#                 count += 1
#                 min_dist = min(min_dist, dist)
#     if count:
#         # return temp_sum / count
#         image = cv2.rectangle(image, (sq_x0, sq_y0), (sq_x1, sq_y1), (255, 255, 255), 2)
#         return min_dist, image
#     image = cv2.rectangle(image, (sq_x0, sq_y0), (sq_x1, sq_y1), (255, 255, 255), 2)
#     return get_dist(depth_frame, perc + 3, image)

def get_dist_in_area(depth_fr, x0, x1, y0, y1):
    count = 0
    temp_sum = 0
    min_dist = 400

    for i in range(x0, x1):
        for j in range(y0, y1):
            dist = depth_fr.get_distance(i, j)
            if dist:
                temp_sum += dist
                count += 1
                min_dist = min(min_dist, dist)
    return count, min_dist


def get_rect_frame(depth_fr, perc, image, sq_x0, sq_x1, sq_y0, sq_y1):
    if perc > 13:
        return 0, image
    count = 0
    min_dist = 400

    fr_x0 = sq_x0 - int(W * PERCENT_DIF / 200)
    fr_x1 = sq_x1 + int(W * PERCENT_DIF / 200)
    fr_y0 = sq_y0 - int(H * PERCENT_DIF / 200)
    fr_y1 = sq_y1 + int(H * PERCENT_DIF / 200)

    if fr_y0 < 0:
        fr_y0 = 0
    if fr_y1 > 480:
        fr_y1 = 480

    if fr_y0 < sq_y0:
        count_up, min_dist_up = get_dist_in_area(depth_fr, fr_x0, fr_x1, fr_y0, sq_y0)
        count += count_up
        min_dist = min(min_dist, min_dist_up)

    if fr_y1 > sq_y1:
        count_down, min_dist_down = get_dist_in_area(depth_fr, fr_x0, fr_x1, sq_y1, fr_y1)
        count += count_down
        min_dist = min(min_dist, min_dist_down)


    count_left, min_dist_left = get_dist_in_area(depth_fr, fr_x0, sq_x0, sq_y0, sq_y1)
    count_right, min_dist_right = get_dist_in_area(depth_fr, sq_x1, fr_x1, sq_y0, sq_y1)
    count += count_left + count_right
    min_dist = min(min_dist, min_dist_left, min_dist_right)

    if count:
        # return temp_sum / count
        image = cv2.rectangle(image, (fr_x0, fr_y0), (fr_x1, fr_y1), (255, 255, 255), 2)
        # print(perc)
        # image = cv2.rectangle(image, (fr_x0, fr_y0), (fr_x1, sq_y1), (255, 255, 255), 2)
        return min_dist, image

    return get_rect_frame(depth_fr, perc + PERCENT_DIF, image, fr_x0, fr_x1, fr_y0, fr_y1)


def get_rect(depth_fr, perc, image, y):
    sq_x0 = W // 2 - int(W * perc / 200)
    sq_x1 = W // 2 + int(W * perc / 200)
    sq_y0 = y - int(H * perc / 200)
    sq_y1 = y + int(H * perc / 200)
    if sq_y0 < 0:
        sq_y0 = 0
    if sq_y1 > 480:
        sq_y1 = 480

    count, min_dist = get_dist_in_area(depth_fr, sq_x0, sq_x1, sq_y0, sq_y1)
    if count:
        image = cv2.rectangle(image, (sq_x0, sq_y0), (sq_x1, sq_y1), (255, 255, 255), 2)
        return min_dist, image
    # return 999999999, image
    return get_rect_frame(depth_fr, perc + PERCENT_DIF, image, sq_x0, sq_x1, sq_y0, sq_y1)

# def zero_frame(depth_frame, image):
#     for i in range(sq_x0 - 20, sq_x1 + 20):
#         for j in range(sq_y0 - 20, sq_y1 + 20):
#             dist = depth_frame.get_distance(i, j)
#             if dist == 0:
#                 image = cv2.circle(image, (i, j), radius=0, color=(0, 0, 255), thickness=-1)
#     return image

config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

# Start streaming
def main():
    angle_x = None
    last_ts_gyro = 0
    pipeline.start(config)
    while True:
        frame = pipeline.wait_for_frames()
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()

        acc = frame[2].as_motion_frame().get_motion_data()
        gyro = frame[3].as_motion_frame().get_motion_data()
        ts = frame.get_timestamp()
        angle_x, last_ts_gyro = get_angle_x(angle_x, acc, gyro, last_ts_gyro, ts)
        # print(angle_x)

        # distance = depth_frame.get_distance(320, 240)
        # print(distance)

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                                         alpha=0.5), cv2.COLORMAP_JET)
        #
        # distance, image = get_dist(depth_frame, RECT_PERCENT, color_image)
        # print(distance)
        if abs(angle_x) < HALF_FOV:
            temp_y = int((240 + angle_x * 240 / HALF_FOV))
        else:
            temp_y = 240
        # distance = (depth_frame.get_distance(320, temp_y))
        distance, image = get_rect(depth_frame, RECT_PERCENT, depth_cm, temp_y)
        if abs(angle_x) < HALF_FOV and distance:
            distance = distance + abs(distance * (angle_x / HALF_FOV * 0.2))
        # distance = distance + abs(distance * (angle_x / HALF_FOV * 0.2))
        # image = cv2.circle(color_image, (320, 240), 3, (255, 0, 0), 2)
        # image = cv2.circle(depth_cm, (320, temp_y), 3, (255, 0, 0), 2)
        # image = cv2.rectangle(color_image, (sq_x0,  sq_y0), (sq_x1, sq_y1), (0, 255, 0), 2)
        # image = cv2.rectangle(color_image, (200,  120), (440, 360), (255, 0, 0), 2)
        # image = zero_frame(depth_frame, color_image)

        cv2.putText(image, str(distance), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color = (255, 255, 255))
        cv2.putText(image, str(angle_x), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(255, 255, 255))
        # cv2.putText(depth_cm, "213", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color = (255, 255, 255))
        # cv2.imshow('depth', image)
        cv2.imshow('color', image)

        if cv2.waitKey(1) == ord('q'):
            break

main()