import cv2
import numpy as np
from gym_duckietown.tasks.task_solution import TaskSolution

"""
Observations are single camera images,
as numpy arrays of size (120, 160, 3).
"""

"""
Actions passed to the step() function
should be numpy arrays containing two
numbers between -1 and 1.

These two numbers correspond to forward velocity,
and a steering angle, respectively.
A positive velocity makes the robot go forward,
and a positive steering angle makes the robot turn left.
"""


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    @staticmethod
    def _process_observation(img: np.ndarray) -> float:
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])

        mask_yellow = cv2.inRange(hsv_img, yellow_lower, yellow_upper)

        return mask_yellow.sum() / 255.0

    def _stop(self, env):
        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])

            yellow_ratio = self._process_observation(img)

            if yellow_ratio > 5_000:
                condition = False

            env.render()

    @staticmethod
    def _rotation(env, side: int):
        for _ in range(15):
            _ = env.step([0.3, 1 * side])
            env.render()

    def _maneuver(self, env, number: int = 1):
        self._rotation(env, 1 * number)

        _ = env.step([1, 0])
        env.render()

        self._rotation(env, -1 * number)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        self._stop(env)
        self._maneuver(env)

        # Едем, пока не пропала уточка.
        # На всякий случай ставим ограничение на число шагов.
        for _ in range(15):
            img, reward, done, info = env.step([1, 0])
            yellow_ratio = self._process_observation(img)

            env.render()

            if yellow_ratio < 3_000:
                break

        self._maneuver(env, -1)

        for _ in range(10):
            _ = env.step([1, 0])
            env.render()

        img, _, _, _ = env.step([0, 0])
