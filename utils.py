import time
import math
import numpy as np
import numpy.linalg as npl

import win32con
import win32api

key_map = {
    "0": 96, "1": 97, "2": 98, "3": 99, "4": 100, "5": 101, "6": 102, "7": 103, "8": 104, "9": 105,
    "A": 65, "B": 66, "C": 67, "D": 68, "E": 69, "F": 70, "G": 71, "H": 72, "I": 73, "J": 74,
    "K": 75, "L": 76, "M": 77, "N": 78, "O": 79, "P": 80, "Q": 81, "R": 82, "S": 83, "T": 84,
    "U": 85, "V": 86, "W": 87, "X": 88, "Y": 89, "Z": 90, ".": 110, "-": 109, "TAB": 9, "BACK": 8,
    "INSERT": 0x2D
}

def key_down(key):
    key = key.upper()
    vk_code = key_map[key]
    win32api.keybd_event(vk_code,win32api.MapVirtualKey(vk_code,0),0,0)
 
 
def key_up(key):
    key = key.upper()
    vk_code = key_map[key]
    win32api.keybd_event(vk_code, win32api.MapVirtualKey(vk_code, 0), win32con.KEYEVENTF_KEYUP, 0)

def key_press(key):
    key_down(key)
    time.sleep(0.001)
    key_up(key)

def move_towards(f, target, max_step):
    if abs(f - target) < max_step:
        return target
    return f - max_step * sgn(f - target)

def clamp(num, maxnum, minnum):
    if maxnum < minnum:
        return clamp(num, minnum, maxnum)
    if num > maxnum:
        return maxnum
    elif num < minnum:
        return minnum
    return num

def clamp_mag(vec, maxmag):
    mag = npl.norm(vec)
    if mag > maxmag:
        return vec / mag * maxmag
    return vec

def lerp(vec1, vec2, t):
    return t * vec2 + (1-t) * vec1

def lmap(f, from1, from2, to1, to2):
    return (f - from1) / (from2 - from1) * (to2 - to1) + to1

def sgn(f):
    if f > 0:
        return 1
    elif f < 0:
        return -1
    return 0

def v2(f1, f2):
    return np.array((f1, f2))

def v3(f1, f2, f3):
    return np.array((f1, f2, f3))

def v(vec):
    return np.array(vec)

def normalize(vec):
    return vec / npl.norm(vec)

def q(axis, angle):
    (x, y, z) = axis
    s = math.sin(angle / 2)
    c = math.cos(angle / 2)
    axis = v3(x+.0, y+.0, z+.0)
    axis /= npl.norm(axis)
    return (s * axis[0], s * axis[1], s * axis[2], c)

def rotation_mat(q):
    (x, y, z, w) = q
    return np.mat([
    [1-2*y**2-2*z**2, 2*x*y+2*w*z, 2*x*z-2*w*y],
    [2*x*y-2*w*z, 1-2*x**2-2*z**2, 2*y*z+2*w*x],
    [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x**2-2*y**2]
    ])

def transform(vec, mat):
    res = np.mat(vec) * mat
    return v3(res[0,0], res[0,1], res[0,2])

def angle_around_axis(v1, v2, axis):
    axis = normalize(axis)
    v1 = normalize(np.cross(v1, axis))
    v2 = normalize(np.cross(v2, axis))
    direction = sgn(np.dot(np.cross(v1, v2), axis))
    return direction * math.acos(np.dot(v1, v2))

def normal_angle(deg):
    while deg > 180:
        deg -= 360
    while deg <= -180:
        deg += 360
    return deg

class PID:
    def __init__(self):
        self.ep = True
        self.ei = True
        self.ed = True
        self.kp = 1
        self.ki = 0
        self.kd = 1
        self.sd = 0
        self.diff = 0
        self.integral = 0
        self.integral_limit = 1
        self.error_prev = 0
        self.first = True
        self.second = True
        self.dumpf = None
    
    def start_dump(self, fname):
        self.dumpf = open(fname, "w+")
    
    def update_dump(self, real_time):
        self.dumpf.write("%f\t%f\t%f\t%f\n" % (real_time, self.p_prev, self.i_prev, self.d_prev))
    
    def update(self, error, dt):
        if self.first:
            self.first = False
            self.error_prev = error
        elif self.second:
            self.second = False
            self.diff = (error - self.error_prev) / dt
        
        self.integral += error * dt * self.ki
        self.integral = clamp(self.integral, self.integral_limit, -self.integral_limit)
        self.diff = lerp(self.diff, (error - self.error_prev) / dt, 1-self.sd)
        p = -error * self.kp
        i = -self.integral
        d = -self.diff * self.kd
        self.result = p * (1 if self.ep else 0) + i * (1 if self.ei else 0) + d * (1 if self.ed else 0)
        
        self.p_prev = p
        self.i_prev = i
        self.d_prev = d
        self.error_prev = error
        return self.result


class FreeInertialControl:
    # 假设：几乎无干扰
    def __init__(self):
        self.unified_authority = 1
        self.redundancy = 0.1
        self.kp = 1
        self.kd = 0
    
    def update(self, error, velocity):
        acc = self.unified_authority * (1 - self.redundancy)
        target_vel = math.sqrt(2 * max(abs(error) - 0.5 * deg2rad, 0) * acc) * sgn(-error)
        self.result = (target_vel - velocity) * self.kp - velocity * self.kd
        return self.result

class LimitedAccelerationControl:
    # 假设：几乎无干扰，目标基本固定
    def __init__(self):
        self.unified_authority = 1
        self.redundancy = 0.1
        self.kp = 1
        self.kd = 0
    
    def update(self, error, velocity):
        acc = self.unified_authority * (1 - self.redundancy)
        target_vel = math.sqrt(2 * max(abs(error) - 0.5 * deg2rad, 0) * acc) * sgn(-error)
        self.result = (target_vel - velocity) * self.kp - velocity * self.kd
        return self.result

deg2rad = math.pi / 180
rad2deg = 180 / math.pi