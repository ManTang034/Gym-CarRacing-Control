from collections import deque

import cv2
import gym
import numpy as np
from gym import wrappers
from simple_pid import PID

import controller
import trajectory_control_utils as tc_utils
import vision
from car_racing import CarRacing

MODE = "right"  # [left,middle,right]
# Initialize the environment
env = CarRacing(render_mode="human", MODE=MODE)

# Wrap the environment to record the video
# env = wrappers.RecordVideo(
#     env, video_folder="./video", episode_trigger=lambda x: x % 1 == 0
# )
# Record every episode

vision_module = vision.BEV_Vision(debug=False)
control_module = controller.CarController(debug=False)

pid_velocity = PID(0.005, 0.001, 0.0005, setpoint=30)
pid_steering = PID(0.8, 0.01, 0.06, setpoint=0)
error_velocity_buffer = deque(np.zeros(7), maxlen=7)
error_buffer = deque(np.zeros(10), maxlen=10)
error_buffer_2 = deque(np.zeros(3), maxlen=3)




if __name__ == "__main__":
    env.reset()
    action = np.array([0, 0, 0], dtype=np.float32)  # action=[steering, gas, breaking]
    obs, _, _, _, info = env.step(action)

    for _ in range(1000):
        augmImg, carVelocity_wFrame, carPosition_wFrame, car_heading_angle, v_wFrame = (
            tc_utils.extract_variables(info)
        )

        # Calculate action
        action_ = tc_utils.trajectory_control(
            augmImg,
            pid_steering,
            pid_velocity,
            error_buffer,
            error_buffer_2,
            error_velocity_buffer,
            v_wFrame,
            MODE,
        )
        print("action:", action)

        if (
            action_ is None
        ):  # In the tightest curve we lose intersection of strip with trajectory
            obs, _, _, _, info = env.step(action)
            (
                augmImg,
                carVelocity_wFrame,
                carPosition_wFrame,
                car_heading_angle,
                v_wFrame,
            ) = tc_utils.extract_variables(info)
            continue

        action = action_
        obs, _, _, _, info = env.step(action)

    env.close()
