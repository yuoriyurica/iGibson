from gibson2.envs.locomotor_env import NavigateRandomEnv, HotspotTravEnv
from time import time, sleep
import numpy as np
import gibson2
import os
from gibson2.core.render.profiler import Profiler
import logging
import math
import cv2 as cv
import threading
from gibson2.utils.utils import quatToXYZW, quatFromXYZW
from transforms3d.euler import euler2quat, quat2euler, euler2axangle, axangle2euler
from transforms3d.quaternions import quat2mat, axangle2quat
import io
import json
from scipy.interpolate import splev, splprep


logging.getLogger().setLevel(logging.DEBUG) #To increase the level of logging

def main():
    global recording
    global nav_env
    config_filename = os.path.join(os.path.dirname(gibson2.__file__),
                                   '../examples/configs/turtlebot_demo.yaml')
    nav_env = HotspotTravEnv(config_file=config_filename, mode='gui')
    for j in range(20):
        nav_env.reset()
        path = nav_env.get_shortest_path(entire_path=True)[0]

        s_data = np.array(sorted(path, key=lambda s : s[0]))
        p_x = s_data[:, 0]
        p_y = s_data[:, 1]

        # s_x, s_y = catmull_rom(p_x, p_y, 5)
        tck, u = splprep([p_x, p_y])
        u = np.linspace(0, 1, 50)
        s_x, s_y = np.array(splev(u, tck))
        nav_env.step_visualization(np.array([[s_x[p], s_y[p]] for p in range(len(s_x))]))
        # save_path(str(j) + '.json', path.tolist())
        recording = True
        threading.Thread(target=recorder_core, args=(str(j) + '.avi',)).start()
        for i in range(len(s_x)):
            # with Profiler('Environment action step'):
            pos = nav_env.get_position_of_interest()
            next_pos = [s_x[i], s_y[i], pos[2]]
            next_dir = np.array([next_pos[0] - pos[0], next_pos[1] - pos[1]])
            # print(next_pos)
            orn = nav_env.get_orientation()
            next_orn = new_orientation_from_dir(orn, next_dir)
            state, reward, done, info = nav_env.step((next_pos, next_orn))
            # sleep(1)
            if done:
                logging.info("Episode finished after {} timesteps".format(i + 1))
                break
        recording = False

def save_path(filePath, data):
    with open(filePath, 'w') as outfile:
        json.dump(data, outfile)

def normalize(vec):
    return vec / np.linalg.norm(vec)

def new_orientation_from_dir(orn, next_dir):
    # initial_vec = np.array([0, 1, 0])
    # orn_matrix = quat2mat(quatFromXYZW(orn, 'wxyz'))
    # dir = orn_matrix.dot(initial_vec.T).T
    # rad = math.acos(np.dot(normalize(dir), normalize(next_dir)))

    # orn_euler = quat2euler(quatFromXYZW(orn, 'wxyz'))
    # vec, theta = euler2axangle(*orn_euler)
    # next_orn_euler = axangle2euler(vec, theta + rad)
    # next_orn = euler2quat(*next_orn_euler)

    # # print(vec, rad, theta + rad)

    # # return orn
    # return quatToXYZW(next_orn, 'wxyz')

    initial_dir = np.array([0, 1])
    rad = np.arccos(np.dot(initial_dir, normalize(next_dir)))
    # print(rad)
    next_orn = quatToXYZW(axangle2quat(np.array([0, 0, 1]), rad, is_normalized=True), 'wxyz')

    return next_orn


def get_frame():
    return (nav_env.simulator.viewer.frame * 255)[:256, :].astype(np.uint8)

def recorder_core(filename):
    res = (640, 480)
    fps = 20.0
    # DIVX(.avi) / XVID
    fourcc = cv.VideoWriter_fourcc(*'DIVX')
    out = cv.VideoWriter(filename, fourcc, fps, res)
    while(recording):
        frame = get_frame()
        out.write(frame)
        sleep(1 / fps)
    out.release()

def catmull_rom(p_x, p_y, res):
    """Computes Catmull-Rom Spline for given support points and resolution.
    Args:
        p_x: array of x-coords
        p_y: array of y-coords
        res: resolution of a segment (including the start point, but not the
            endpoint of the segment)
    """
    # create arrays for spline points
    x_intpol = np.empty(res*(len(p_x)-1) + 1)
    y_intpol = np.empty(res*(len(p_x)-1) + 1)

    # set the last x- and y-coord, the others will be set in the loop
    x_intpol[-1] = p_x[-1]
    y_intpol[-1] = p_y[-1]

    # loop over segments (we have n-1 segments for n points)
    for i in range(len(p_x)-1):
        # set x-coords
        x_intpol[i*res:(i+1)*res] = np.linspace(
            p_x[i], p_x[i+1], res, endpoint=False)
        if i == 0:
            # need to estimate an additional support point before the first
            y_intpol[:res] = np.array([
                catmull_rom_one_point(
                    x,
                    p_y[0] - (p_y[1] - p_y[0]), # estimated start point,
                    p_y[0],
                    p_y[1],
                    p_y[2])
                for x in np.linspace(0.,1.,res, endpoint=False)])
        elif i == len(p_x) - 2:
            # need to estimate an additional support point after the last
            y_intpol[i*res:-1] = np.array([
                catmull_rom_one_point(
                    x,
                    p_y[i-1],
                    p_y[i],
                    p_y[i+1],
                    p_y[i+1] + (p_y[i+1] - p_y[i]) # estimated end point
                ) for x in np.linspace(0.,1.,res, endpoint=False)])
        else:
            y_intpol[i*res:(i+1)*res] = np.array([
                catmull_rom_one_point(
                    x,
                    p_y[i-1],
                    p_y[i],
                    p_y[i+1],
                    p_y[i+2]) for x in np.linspace(0.,1.,res, endpoint=False)])


    return (x_intpol, y_intpol)

def catmull_rom_one_point(x, v0, v1, v2, v3):
    """Computes interpolated y-coord for given x-coord using Catmull-Rom.
    Computes an interpolated y-coordinate for the given x-coordinate between
    the support points v1 and v2. The neighboring support points v0 and v3 are
    used by Catmull-Rom to ensure a smooth transition between the spline
    segments.
    Args:
        x: the x-coord, for which the y-coord is needed
        v0: 1st support point
        v1: 2nd support point
        v2: 3rd support point
        v3: 4th support point
    """
    c1 = 1. * v1
    c2 = -.5 * v0 + .5 * v2
    c3 = 1. * v0 + -2.5 * v1 + 2. * v2 -.5 * v3
    c4 = -.5 * v0 + 1.5 * v1 + -1.5 * v2 + .5 * v3
    return (((c4 * x + c3) * x + c2) * x + c1)

if __name__ == "__main__":
    main()
