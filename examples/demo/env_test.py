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
from pyquaternion import Quaternion


#logging.getLogger().setLevel(logging.DEBUG) #To increase the level of logging

def main():
    global recording
    global nav_env
    config_filename = os.path.join(os.path.dirname(gibson2.__file__),
                                   '../examples/configs/turtlebot_demo.yaml')
    nav_env = HotspotTravEnv(config_file=config_filename, mode='gui')
    for j in range(10):
        nav_env.reset()
        path = nav_env.get_shortest_path(entire_path=True)[0]
        catmull_x, catmull_y = catmull_rom(path[:, 0], path[:, 1], 5)
        print(len(path), len(catmull_x))
        recording = True
        # threading.Thread(target=recorder_core, args=(str(j) + '.avi',)).start()
        for i in range(500):
            with Profiler('Environment action step'):
                pos = nav_env.get_position_of_interest()
                next_pos = [catmull_x[i], catmull_y[i], pos[2]]
                next_dir = np.array([next_pos[0] - pos[0], next_pos[1] - pos[1], 0.0])

                orn = nav_env.get_orientation()
                qorn = Quaternion(array=np.roll(orn, 1))
                next_qorn = new_orientation_from_dir(qorn, next_dir)
                next_orn = np.array([next_qorn[1], next_qorn[2], next_qorn[3], next_qorn[0]])
                state, reward, done, info = nav_env.step((next_pos, next_orn))
                if done:
                    logging.info("Episode finished after {} timesteps".format(i + 1))
                    break
        recording = False

def normalize(vec):
    return vec / np.linalg.norm(vec)

def new_orientation_from_dir(qorn, next_dir):
    initial_vec = np.array([0.0, 1.0, 0.0])
    dir = qorn.rotate(initial_vec)
    rad = math.acos(np.dot(normalize(dir), normalize(next_dir)))
    final_rad = rad + qorn.radians - math.floor((rad + qorn.radians) / math.pi) * math.pi
    print(final_rad - qorn.radians)
    return Quaternion(axis=(0.0, 0.0, 1.0), radians=final_rad) if abs(final_rad - qorn.radians) < math.pi / 6 else qorn


def get_frame():
    return (nav_env.simulator.viewer.frame * 255).astype(np.uint8)

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
