# Gym-CarRacing-Control
**Objective:**
The goal of this project is to implement a PID (Proportional-Integral-Derivative) controller to enable a car in the CarRacing environment from OpenAI's Gym library to follow a predefined path or track. 

| ![left_image](videos/left_image.png) | ![middle_image](videos/middle_image.png) | ![right_image](videos/right_image.png) |
| :----------------------------------: | :--------------------------------------: | :------------------------------------: |
|       ![left](videos/left.gif)       |       ![middle](videos/middle.gif)       |       ![right](videos/right.gif)       |
|              left track              |               middle track               |              right track               |

------

## 1.Trajectory Masking

- **Input: **RGB image of the environment(96*96)
- **Output:** masks for left, middle, and right trajectories
- **Steps:**
  - Define HSV color ranges for each trajectory:
    - Cyan: Left side of the track.
    - Magenta: Middle of the track.
    - Blue: Right side of the track.
  - Apply `cv2.inRange()` to generate masks

## 2.Target Point Calculation

- **Input:** masked image, strip distance, and trajectory mode
- **Output:** target point coordinates
- **Steps:**
  - Extract the relevant mask based on `MODE`
  - Identify non-zero pixels in the strip
  - Compute the target point as the closest pixel to the center

## 3.Error and Steering Angle Calculation

**Input:** target point, car position

**Output:** error, angle
$$
error=target\_point-car\_position
$$

$$
angle=arctan(error/car2point\_vector)
$$

## 4.Control Action Calculation

- **Input:** Steering angle, velocity error, PID controllers

- **Output:** Control actions

- **Steps:**
  $$
  action\_steering=pid\_steering(angle)
  $$

  $$
  action\_gas=pid\_velocity(error\_vel)
  $$

  $$
  action\_brake=clip(action\_velocity,0,0.9)
  $$

------

