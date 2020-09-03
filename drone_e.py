import krpc
import time
import math
import numpy as np
import numpy.linalg as npl
import sys

#utils.py
from utils import *

def create_target(body, lat_deg, lon_deg, height_sealevel):
    lat_rad = lat_deg * deg2rad
    lon_rad = lon_deg * deg2rad
    height_cm = height_sealevel + body.equatorial_radius
    target_body_pos = np.array((math.cos(lon_rad) * math.cos(lat_rad), math.sin(lat_rad), math.sin(lon_rad) * math.cos(lat_rad))) * height_cm
    ref_body = body.reference_frame
    return space_center.ReferenceFrame.create_relative(ref_body, position=target_body_pos)


conn = krpc.connect(name='krpc_demo')
space_center = conn.space_center
vessel = space_center.active_vessel
flight = vessel.flight()
body = vessel.orbit.body



ref_local = vessel.reference_frame 
ref_surface = vessel.surface_reference_frame #地面参考系
ref_body = body.reference_frame
#ref_hybrid = space_center.ReferenceFrame.create_hybrid(ref_surface, rotation=ref_body, velocity=ref_body, angular_velocity=ref_body)
ref_target_temp = create_target(body, flight.latitude, flight.longitude, 50)
ref_target = space_center.ReferenceFrame.create_hybrid(ref_target_temp, rotation=ref_surface, velocity=ref_target_temp)

#各种目标
target_alt = 90
target_alt_vel = 0
target_pit = 0 * deg2rad
target_yaw = 0 * deg2rad
target_rol = 0 * deg2rad
target_rol_avel = 0 * deg2rad
mode_alt = 'vel'
mode_rol = 'vel'

#高度
pid_alt = PID()
pid_alt.kp = 0.1
pid_alt.kd = 0.15
pid_alt.ki = 0.02
pid_alt.integral_limit = 0.2

pid_alt_vel = PID()
pid_alt_vel.kp = 0.2
pid_alt_vel.kd = 0
pid_alt_vel.ki = 0.2
pid_alt_vel.integral_limit = 0.2

#前后倾斜
pid_pit = PID()
pid_pit.kp = 1.2
pid_pit.kd = 0.4

#左右倾斜
pid_yaw = PID()
pid_yaw.kp = 1.2
pid_yaw.kd = 0.4

#转向
pid_rol = PID()
pid_rol.kp = 0.8
pid_rol.kd = 0.3
pid_rol.ki = 0.8
pid_rol.integral_limit = 0.12

pid_rol_avel = PID()
pid_rol_avel.kp = 0.2
pid_rol_avel.kd = 0
pid_rol_avel.ki = 2.0
pid_rol_avel.integral_limit = 0.12

#水平移动控制
pid_target_f = PID()
pid_target_f.kp = 0.1
pid_target_f.kd = 0.14
pid_target_r = PID()
pid_target_r.kp = 0.1
pid_target_r.kd = 0.14

g0 = 9.81
delta_time = 0.01
game_delta_time = 0.02
game_prev_time = space_center.ut

#start_time = time.time()


    #rotor.01s
    #smallPropeller

    #ModuleRoboticServoRotor
    #ModuleControlSurface

    #Torque Limit(%)
    #RPM Limit

    #Deploy Angle

#for part in vessel.parts.with_name('smallPropeller'):
#    for m in part.modules:
#        print(m.name)


#for module in vessel.parts.modules_with_name('ModuleControlSurface'):
#    for key in module.fields:
#        print('%s : %s' % (key, module.fields[key]))
#    break

engines = {'l' : {'blades' : []}, 'r' : {'blades' : []}, 'f' : {'blades' : []}, 'b' : {'blades' : []}} #用来储存转轴和桨叶的引用
for servo in vessel.parts.modules_with_name('ModuleRoboticServoRotor'): #找到转轴并存好
    servo.set_field_float('Torque Limit(%)', 5.0)
    part = servo.part
    pos = part.position(ref_local)
    if pos[0] > abs(pos[2]): #right
        engines['r']['servo'] = servo
    elif pos[0] < -abs(pos[2]): #left
        engines['l']['servo'] = servo
    elif pos[2] > abs(pos[0]): #f
        engines['f']['servo'] = servo
    elif pos[2] < -abs(pos[0]): #back
        engines['b']['servo'] = servo
    else:
        continue

for blade in vessel.parts.modules_with_name('ModuleControlSurface'): #找到桨叶并存好
    part = blade.part
    pos = part.position(ref_local)
    if pos[0] > abs(pos[2]): #right
        engines['r']['blades'].append(blade)
    elif pos[0] < -abs(pos[2]): #left
        engines['l']['blades'].append(blade)
    elif pos[2] > abs(pos[0]): #forward
        engines['f']['blades'].append(blade)
    elif pos[2] < -abs(pos[0]): #back
        engines['b']['blades'].append(blade)
    else:
        continue

#print(engines)



def thr_set(direction, thr): #设置桨距
    thr = clamp(thr, -1, 1)
    for blade in engines[direction]['blades']:
        blade.set_field_float('Deploy Angle', lmap(thr, -1, 1, -9, 9))

def rpm_set(direction, rpm): #设置转速
    rpm = clamp(rpm, 200, 460)
    engines[direction]['servo'].set_field_float('RPM Limit', rpm)

def combine_thr(pitch_up, yaw_right, turn_right, throttle): #根据给定的控制参数设置各桨距和转速
    
    thr_set('f', throttle + pitch_up)
    thr_set('b', throttle - pitch_up)
    thr_set('l', throttle + yaw_right)
    thr_set('r', throttle - yaw_right)
    rpm_set('f', lmap(turn_right, -1, 1, 200, 460))
    rpm_set('b', lmap(turn_right, -1, 1, 200, 460))
    rpm_set('l', lmap(-turn_right, -1, 1, 200, 460))
    rpm_set('r', lmap(-turn_right, -1, 1, 200, 460))
    

combine_thr(0, 0, 0, 0)
#sys.exit(0)

while True:
    time.sleep(delta_time)
    ut = space_center.ut
    game_delta_time = ut - game_prev_time
    if game_delta_time < 0.001: #意味着游戏中还没有经过一个物理帧，所以不进行计算
        continue
    
    #获得当前飞行状态的各种参数
    avel = v(vessel.angular_velocity(ref_target)) #角速度
    vel = v(vessel.velocity(ref_target)) #速度
    
    rotation_local2srf = rotation_mat(v(vessel.rotation(ref_surface))) # 机体系到地面系旋转矩阵
    rotation_srf2local = npl.inv(rotation_local2srf) # 地面系到机体系旋转矩阵
    
    alt = flight.mean_altitude #海平面高度
    mass = vessel.mass #质量
    
    vel_norm = npl.norm(vel)
    vel_hor = npl.norm(vel[1:3]) #水平速度
    vel_hdg = math.atan2(vel[2], vel[1]) #航向
    vel_vertical = vel[0] #垂直速度
    vel_fwd = v3(0, math.cos(vel_hdg), math.sin(vel_hdg)) #水平速度方向
    vel_right = v3(0, -math.sin(vel_hdg), math.cos(vel_hdg)) #水平速度右侧方向
    vel_up = v3(1, 0, 0)
    body_fwd = transform(v3(0.0, 0.0, 1.0), rotation_local2srf) #机体前方指向
    body_right = transform(v3(1.0, 0.0, 0.0), rotation_local2srf) #机体右方指向
    body_up = transform(v3(0.0, 1.0, 0.0), rotation_local2srf) #机体上方指向
    body_hdg = math.atan2(body_fwd[2], body_fwd[1]) #机头角度
    body_fwd_hor = v3(0, math.cos(body_hdg), math.sin(body_hdg)) #机体前方水平面内指向
    body_right_hor = v3(0, -math.sin(body_hdg), math.cos(body_hdg)) #机体右方水平面内指向
    
    #用户输入
    input_forward = vessel.control.forward
    input_pitch = vessel.control.pitch
    input_yaw = vessel.control.yaw
    input_roll = vessel.control.roll
    
    #当前状态
    current_alt = alt
    current_alt_vel = vel_vertical
    current_pit = math.asin(body_fwd[0])
    current_yaw = math.asin(-body_right[0])
    current_rol = body_hdg
    current_rol_avel = np.dot(v3(1.0, 0.0, 0.0), avel)
    
    #目标
    if mode_alt == 'vel':
        target_alt_vel = input_forward * 5
        target_alt = current_alt
    else:
        target_alt += input_forward * 5 * game_delta_time
    if mode_rol == 'vel':
        target_rol_avel = input_roll * 150 * deg2rad
        target_rol = current_rol
    else:
        target_rol += input_roll * 2 * game_delta_time
    target_vessel = space_center.target_vessel
    if target_vessel == None:
        #如果没有target就由玩家控制倾斜
        target_pit = input_pitch * 30 * deg2rad
        target_yaw = input_yaw * 30 * deg2rad
    else: # 套 娃 PID
        #如果有target就自动飞到target正上方
        error_target = target_vessel.position(ref_surface)
        error_target_f = -np.dot(body_fwd_hor, error_target)
        error_target_r = -np.dot(body_right_hor, error_target)
        print(error_target_f, error_target_r)
        control_target_f = pid_target_f.update(error_target_f, game_delta_time)
        control_target_r = pid_target_r.update(error_target_r, game_delta_time)
        target_pit = -clamp(control_target_f, -1.0, 1.0) * 20 * deg2rad
        target_yaw = clamp(control_target_r, -1.0, 1.0) * 20 * deg2rad
    
    #计算偏差
    error_alt = current_alt - target_alt
    error_alt_vel = current_alt_vel - target_alt_vel
    error_pit = current_pit - target_pit
    error_yaw = current_yaw - target_yaw
    error_rol = normal_angle((current_rol - target_rol) * rad2deg) * deg2rad
    error_rol_avel = current_rol_avel - target_rol_avel
    
    
    #计算控制
    weight = 0.15 / body_up[0] * (mass / 590)
    control_throttle = weight + (pid_alt_vel.update(error_alt_vel, game_delta_time) if mode_alt == 'vel' else pid_alt.update(error_alt, game_delta_time))
    control_pit = pid_pit.update(error_pit, game_delta_time)
    control_yaw = pid_yaw.update(error_yaw, game_delta_time)
    control_rol_avel = pid_rol_avel.update(error_rol_avel, game_delta_time) if mode_rol == 'vel' else pid_rol.update(error_rol, game_delta_time)
    
    #控制
    combine_thr(control_pit, control_yaw, control_rol_avel, control_throttle)
    
    
    
    game_prev_time = ut
    