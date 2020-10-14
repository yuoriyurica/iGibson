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
from scipy.interpolate import splev, splprep, Rbf, InterpolatedUnivariateSpline, CubicSpline, interp1d
from CatmullRomSpline import CatmullRomChain, RecursiveCatmullRomChain
from geomdl import BSpline, fitting

logging.getLogger().setLevel(logging.DEBUG) #To increase the level of logging

def smooth_filter(p_x, p_y):
    size = 2 # filter range
    
    x = []
    y = []
    for i in range(size):
        x.append(p_x[i])
        y.append(p_y[i])

    for i in range(size, len(p_x) - size, 1):
        new_x = 0
        new_y = 0
        for j in range(size * 2 + 1):
            idx = j - size
            new_x += p_x[i + idx]
            new_y += p_y[i + idx]

        new_x /= (size * 2 + 1)
        new_y /= (size * 2 + 1)

        x.append(new_x)
        y.append(new_y)

    for i in range(size):
        idx = i - size
        x.append(p_x[idx])
        y.append(p_y[idx])
    return np.array(x), np.array(y)

def main():
    global recording
    global nav_env
    config_filename = os.path.join(os.path.dirname(gibson2.__file__),
                                   '../examples/configs/turtlebot_demo.yaml')
    nav_env = HotspotTravEnv(config_file=config_filename, mode='gui')
    for j in range(20):
        nav_env.reset()
        path, dist = nav_env.get_shortest_path(entire_path=True)
        print(dist)

        p_x = path[:, 0]
        p_y = path[:, 1]
        p_t = np.arange(len(p_x))

        c = CatmullRomChain(path)
        s_x, s_y = zip(*c)

        # tck_x, u_x = splprep([p_t, p_x], k=5)
        # tck_y, u_y = splprep([p_t, p_y], k=5)
        # u = np.linspace(0, 1, 100)
        # _, s_x = np.array(splev(u, tck_x))
        # _, s_y = np.array(splev(u, tck_y))

        # rbf_x = Rbf(p_t, p_x)
        # rbf_y = Rbf(p_t, p_y)
        # u = np.linspace(0, len(p_t), 99)
        # s_x = rbf_x(u)
        # s_y = rbf_y(u)

        # ius_x = InterpolatedUnivariateSpline(p_t, p_x)
        # ius_y = InterpolatedUnivariateSpline(p_t, p_y)
        # u = np.linspace(0, len(p_t), 99)
        # s_x = ius_x(u)
        # s_y = ius_y(u)

        f_x = interp1d(p_t, p_x)
        f_y = interp1d(p_t, p_y)
        _u = np.linspace(0, p_t[-1], len(p_t) * 1)
        # _p_x, _p_y = smooth_filter(f_x(_u), f_y(_u))
        _p_x, _p_y = smooth_filter(np.array(s_x), np.array(s_y))

        _p_t = np.arange(len(_p_x))
        cs_x = CubicSpline(_p_t, _p_x)
        cs_y = CubicSpline(_p_t, _p_y)
        u = np.linspace(0, _p_t[-1], int(dist * 30))
        s_x = cs_x(u)
        s_y = cs_y(u)

        # crv = fitting.interpolate_curve(path.tolist(), 5)
        # u = np.linspace(0, 1, 100)
        # pts = np.array(crv.evaluate_list(u))
        # s_x = pts[:, 0]
        # s_y = pts[:, 1]

        nav_env.step_visualization(np.array([[s_x[p], s_y[p]] for p in range(len(s_x))]))
        recording = True
        # threading.Thread(target=recorder_core, args=(os.path.join(os.path.dirname(gibson2.__file__), '../examples/captures', 'bs'+str(j) + '.avi'),)).start()
        # save_path(str(j) + '.json', path.tolist())
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
            if i == len(s_x) - 1:
                logging.info("Episode finished after {} timesteps".format(i + 1))
                break
        recording = False

def save_path(filePath, data):
    with open(filePath, 'w') as outfile:
        json.dump(data, outfile)

def normalize(vec):
    return vec / np.linalg.norm(vec)

def new_orientation_from_dir(orn, next_dir):
    initial_dir = np.array([1, 0])
    cos = np.dot(initial_dir, normalize(next_dir))
    sin = np.cross(initial_dir, normalize(next_dir))
    rad = np.arccos(cos)
    
    if sin < 0:
        rad = -rad

    # print(rad)
    next_orn = quatToXYZW(axangle2quat(np.array([0, 0, 1]), rad, is_normalized=True), 'wxyz')

    return next_orn


def get_frame():
    return (nav_env.simulator.viewer.frame * 255).astype(np.uint8)

def get_clipped(frame):
    return get_frame()[:256, :]

def recorder_core(filename):
    res = (512, 512)
    fps = 20.0
    # DIVX(.avi) / XVID
    fourcc = cv.VideoWriter_fourcc(*'DIVX')
    out = cv.VideoWriter(filename, fourcc, fps, res)
    while(recording):
        frame = get_frame()
        out.write(frame)
        sleep(1 / fps)
    out.release()

if __name__ == "__main__":
    main()
