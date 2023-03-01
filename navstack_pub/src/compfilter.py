import numpy as np
import math

class imu6Dof:

    def __init__(self):

        # complementary filter constants
        self.alpha = 0.95
        
        self.roll_r = 0.0
        self.pitch_r = 0.0
        
        self.roll = 0.0
        self.pitch = 0.0

    def update(self, ax, ay, az, gx, gy, gz, dt):
        
        

        # calculate accelerometer angles
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    
        roll_gyro = self.roll_r - gx * dt
        pitch_gyro = self.pitch_r - gy * dt

        self.roll_r = self.alpha*roll_gyro + (1-self.alpha)*roll_acc
        self.pitch_r = self.alpha*pitch_gyro + (1-self.alpha)*pitch_acc
        
        self.roll = math.degrees(self.roll_r)
        self.pitch = math.degrees(self.pitch_r)
