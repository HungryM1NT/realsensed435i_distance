import pyrealsense2 as rs
import numpy as np
import cv2
from pyrealsense2 import motion_frame

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
angle_x = None
last_ts_gyro = 0

def get_dist(depth_frame, perc, image):
    if perc > 13:
        return "Zero", image
    count = 0
    temp_sum = 0
    min_dist = 120
    sq_x0 = W // 2 - int(W * perc / 200)
    sq_x1 = W // 2 + int(W * perc / 200)
    sq_y0 = H // 2 - int(H * perc / 200)
    sq_y1 = H // 2 + int(H * perc / 200)
    for i in range(sq_x0, sq_x1):
        for j in range(sq_y0, sq_y1):
            dist = depth_frame.get_distance(i, j)
            if dist:
                temp_sum += dist
                count += 1
                min_dist = min(min_dist, dist)
    if count:
        # return temp_sum / count
        image = cv2.rectangle(image, (sq_x0, sq_y0), (sq_x1, sq_y1), (255, 255, 255), 2)
        return min_dist, image
    image = cv2.rectangle(image, (sq_x0, sq_y0), (sq_x1, sq_y1), (255, 255, 255), 2)
    return get_dist(depth_frame, perc + 3, image)

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

    # depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    # depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
    #                                                  alpha=0.5), cv2.COLORMAP_JET)
    #
    # distance, image = get_dist(depth_frame, RECT_PERCENT, color_image)
    # print(distance)
    if abs(angle_x) < 21:
        temp_y = int((240 + angle_x * 240 / 21))
    else:
        temp_y = 240
    distance = (depth_frame.get_distance(320, temp_y))
    distance = distance + abs(distance * (angle_x / 21 * 0.2))
    # image = cv2.circle(color_image, (320, 240), 3, (255, 0, 0), 2)
    image = cv2.circle(color_image, (320, temp_y), 3, (255, 0, 0), 2)
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
