import math

import cv2
import numpy as np


def maskTrajectories(image):
    # Define the threshold ranges for each mask
    lower_cyan = np.array([0, 100, 100], dtype=np.uint8)  ## Left side of track on road
    upper_cyan = np.array([0, 255, 255], dtype=np.uint8)
    lower_magenta = np.array([100, 0, 100], dtype=np.uint8)  ## Middle of track on road
    upper_magenta = np.array([255, 0, 255], dtype=np.uint8)
    lower_blue = np.array([0, 0, 100], dtype=np.uint8)  ## Right of track in grass
    upper_blue = np.array([0, 0, 255], dtype=np.uint8)

    # Apply the masks, mask-> 1 channel
    mask_blue = cv2.inRange(image, lower_blue, upper_blue)
    mask_cyan = cv2.inRange(image, lower_cyan, upper_cyan)
    mask_magenta = cv2.inRange(image, lower_magenta, upper_magenta)

    # Label the differently colored trajectories
    dict_masks = {
        "left": mask_cyan,
        "middle": mask_magenta,
        "right": mask_blue,
    }
    return dict_masks


def calculate_target_point(augmImg, strip_distance, MODE):
    dict_masks = maskTrajectories(
        augmImg
    )  # Get dictionary of masks containing each trajectory
    track_img = dict_masks[MODE]  # Get the mask of the trajectory we are following
    line_strip = track_img[strip_distance, :]
    idx = np.nonzero(line_strip)[0]
    # print("idx:", idx)

    if len(idx) == 0:
        return None

    idx = idx[np.argmin(np.abs(idx - 48))]
    target_point = np.array([strip_distance, idx])
    return target_point


def calculate_car2point_vector(target_point, car_pos_vector):
    return target_point - car_pos_vector


def calculate_steering_angle(error_avg_2, car2point_vector):
    angle = np.arctan2(abs(error_avg_2), abs(car2point_vector[0]))
    if error_avg_2 > 0:
        angle = -angle
    return angle


def calculate_steering_action(angle, pid_steering):
    return pid_steering(angle)


def calculate_velocity_error(pid_velocity, v_wFrame):
    return pid_velocity.setpoint - np.linalg.norm(v_wFrame)


def calculate_velocity_action(error_vel_avg, pid_velocity, v_wFrame):
    if error_vel_avg < 0:
        return 0, np.clip(
            np.linalg.norm(pid_velocity(np.linalg.norm(v_wFrame))), 0, 0.9
        )
    else:
        return pid_velocity(np.linalg.norm(v_wFrame)), 0


def trajectory_control(
    augmImg,
    pid_steering,
    pid_velocity,
    error_buffer,
    error_buffer_2,
    error_velocity_buffer,
    v_wFrame,
    MODE,
):

    strip_distance = (
        60  # x - coordinates of the strip, ie image row starting in upper left corner
    )
    car_pos_vector = np.array([70, 48])  # [y,x]

    target_point = calculate_target_point(augmImg, strip_distance, MODE)
    print("target_point:", target_point)
    if target_point is None:
        return None

    car2point_vector = calculate_car2point_vector(target_point, car_pos_vector)
    err = target_point[1] - 48.0
    err = np.clip(err, -5, 5)

    if np.linalg.norm(err) <= 2:
        err = 0.3 * err

    error_buffer.append(err)
    error_avg = sum(error_buffer) / len(error_buffer)
    error_buffer_2.append(error_avg)
    error_avg_2 = sum(error_buffer_2) / len(error_buffer_2)

    angle = calculate_steering_angle(error_avg_2, car2point_vector)
    action_steering = calculate_steering_action(angle, pid_steering)

    error_vel = calculate_velocity_error(pid_velocity, v_wFrame)
    if np.linalg.norm(error_vel) < 2.0:
        error_vel = 0.0 * error_vel  # Attenuate the error if it is too small

    error_velocity_buffer.append(error_vel)
    error_vel_avg = sum(error_velocity_buffer) / len(error_velocity_buffer)
    action_velocity, action_acceleration = calculate_velocity_action(
        error_vel_avg, pid_velocity, v_wFrame
    )

    return [action_steering, action_velocity, action_acceleration]


def extract_variables(info):
    augmImg = info["augmented_img"]
    velB2vec = info["car_velocity_vector"]
    posB2vec = info["car_position_vector"]
    car_heading_angle = info["car_init_angle"]

    carVelocity_wFrame = [velB2vec.x, velB2vec.y]
    carPosition_wFrame = [posB2vec.x, posB2vec.y]
    v_wFrame = np.linalg.norm(velB2vec)

    return augmImg, carVelocity_wFrame, carPosition_wFrame, car_heading_angle, v_wFrame

