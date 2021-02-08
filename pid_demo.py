import random
import numpy as np
import matplotlib.pyplot as plt
import math

class robot(object):
    def __init__(self, length=20.0):
        """
        Creates robot and initializes location/orientation to 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, x, y, orientation):
        """
        Sets a robot coordinate.
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        """
        Sets the noise parameters.
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Sets the systematical steering drift parameter
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        steering = front wheel steering angle, limited by max_steering_angle
        distance = total distance driven, most be non-negative
        """
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0
        # apply noise for random noise

        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length
        if abs(turn) < tolerance:

            # approximate by straight line motion
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:

            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


#--------------Proportional Control----------------
#Proportional Control考虑当前偏差，偏差越大就让车辆越快的向中心线靠拢。

# def run(param):
#     x_trajectory = []
#     y_trajectory = []
#     myrobot = robot()
#     myrobot.set(0.0, 1.0, 0.0)
#     print('start:',myrobot)
#     speed = 3.0 # motion distance is equalt to speed (we assume time = 1)
#     N = 20
#     for i in range(N):
#         crosstrack_error = myrobot.y
#         steer = -param * crosstrack_error #the main formula
#         print('{}: {}'.format(i,steer))
#         myrobot.move(steer, speed)

#         print(i,':',myrobot)
#         print(' ')
#         x_trajectory.append(myrobot.x)
#         y_trajectory.append(myrobot.y)
#     return x_trajectory, y_trajectory

# x_trajectory, y_trajectory = run(0.1)
# n = len(x_trajectory)
# plt.plot(x_trajectory, y_trajectory, 'g', label='Proportional Control')
# plt.plot(x_trajectory, np.zeros(n), 'r', label='reference')
# plt.plot(x_trajectory, y_trajectory, 'o')
# plt.legend()
# plt.show()




# --------------P&D Control----------------

robot = robot()
robot.set(0, 1, 0)
def run(robot, tau_p, tau_d, n=150, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    crosstrack_error = robot.y
    for i in range(n):

        diff_crosstrack_error = robot.y - crosstrack_error #the main formula
        steer = -tau_p * crosstrack_error - tau_d * diff_crosstrack_error  #the main formula 

        crosstrack_error = robot.y

        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory



x_trajectory, y_trajectory = run(robot, 0.1, 3.0)
n = len(x_trajectory)
plt.subplot(3, 1, 1)
plt.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
plt.plot(x_trajectory, np.zeros(n), 'r', label='reference')
plt.plot(x_trajectory, y_trajectory, 'o')
plt.legend()

x_trajectory, y_trajectory = run(robot, 0.2, 3.0)
n = len(x_trajectory)
plt.subplot(3, 1, 2)
plt.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
plt.plot(x_trajectory, np.zeros(n), 'r', label='reference')
plt.plot(x_trajectory, y_trajectory, 'o')
plt.legend()

x_trajectory, y_trajectory = run(robot, 0.3, 3.0)
n = len(x_trajectory)
plt.subplot(3, 1, 3)
plt.plot(x_trajectory, y_trajectory, 'g', label='PD controller')
plt.plot(x_trajectory, np.zeros(n), 'r', label='reference')
plt.plot(x_trajectory, y_trajectory, 'o')
plt.legend()

plt.show()




# #--------------PID Control----------------
# robot = robot()
# robot.set(0, 1, 0)
# robot.set_steering_drift(10.0 * math.pi / 180.0)

# def run(robot, tau_p, tau_d, tau_i, n=200, speed=1.0):
#     x_trajectory = []
#     y_trajectory = []
#     int_crosstrack_error = 0
#     crosstrack_error = robot.y
#     for i in range(n):

#         diff_crosstrack_error = robot.y - crosstrack_error #the main formula

#         crosstrack_error = robot.y

#         int_crosstrack_error += crosstrack_error #the main formula
#         steer = -tau_p * crosstrack_error - tau_d * diff_crosstrack_error -tau_i * int_crosstrack_error #the main formula
        
        
#         robot.move(steer, speed)
#         x_trajectory.append(robot.x)
#         y_trajectory.append(robot.y)
        
#         print(robot) # print the repr

#     return x_trajectory, y_trajectory


# x_trajectory, y_trajectory = run(robot, 0.2, 3.0, 0.004)
# n = len(x_trajectory)


# plt.plot(x_trajectory, y_trajectory, 'g', label='PID controller')
# plt.plot(x_trajectory, np.zeros(n), 'r', label='reference')
# plt.legend()
# plt.show()


