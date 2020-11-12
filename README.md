# Joshua-Barr-Lane-Keeping-Control-Labs-1


import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt  # "as plt" saves us from writing matplotlib.pyplot
class Car:

   def __init__(self, length=2.3, velocity=5, x=0, y=0, pose=0):
       self.__length = length
       self.__velocity = velocity
       self.__x = x
       self.__y = y
       self.__pose = pose
   def move(self, steering_angle, dt):

# Simulation of the trajectory motion of the
# the car from t=0, to time t=0+dt starting
# from z_initial

       def bicycle_model(t, z):
           theta = z[2]
           return [self.__velocity * np.cos(theta),
                   self.__velocity * np.sin(theta),
                   self.__velocity * np.tan(steering_angle) / self.__length]
       z_initial = [self.__x, self.__y, self.__pose]
       solution = solve_ivp(bicycle_model,
                            [0, dt],
                            z_initial)

       self.__x = solution.y[0][-1]
       self.__y = solution.y[1][-1]
       self.__pose = solution.y[2][-1]
   def x(self):
       return self.__x
   def y(self):
       return self.__y
   def pose(self):
       return self.__pose
   def length(self):
       return self.__length
   def velocity(self):
       return self.__velocity
class PIDController:

#Attributes:
# Kp - Proprtional gain
# Kd - Derivative gain
# Ki - Intergral gain
# Ts - Sampling Time

   def __init__(self, kp, ki, kd, ts):
       """
       Constructor for PIDContoller
       :param kp: proportional gain
       :param ki: integral gain
       :param kd: derivative gain
       :param ts: sampling gain
       """

       self.__kp = kp
       self.__kd = kd / ts
       self.__ki = ki * ts
       self.__ts = ts
       self.__previous_error = None
       self.__sum_errors = 0

   def control(self, y, y_set_point=0):
       error = y_set_point - y
       control_action = self.__kp * error

       if self.__previous_error is not None:
          control_action += self.__kd * (error - self.__previous_error)
       control_action += self.__ki * self.__sum_errors
       self.__sum_errors += error
       self.__previous_error = error
       return control_action


# -----------------------------------------------------------
t_sampling = 0.05
pid = PIDController(kp=0.5, ki=0.025, kd=0.4, ts=t_sampling)
murphy = Car(x=0, y=0.5, pose=0)

y_cache = np.array([murphy.y()]) # The current value of y
num_points = 500                 # was inserted into the cache


for k in range(num_points):
   control_action = pid.control(y=murphy.y())
   murphy.move(control_action + 0.01, t_sampling)
   y_cache = np.append(y_cache, murphy.y())

t_span = t_sampling * np.arange(num_points + 1)

plt.plot(t_span, y_cache)
plt.title("PID Controller  Kp-0.5   Kd-0.4   Ki-0.1")
plt.xlabel("Time (Seconds)")
plt.ylabel("Lateral position, y (Meters) Y")
plt.grid()
plt.show()
