import cv2
import sys
import os
import numpy as np
from gibson2.core.render.mesh_renderer.mesh_renderer_cpu import MeshRenderer
from gibson2.core.render.profiler import Profiler
from gibson2.utils.assets_utils import get_model_path
from gibson2.core.physics.scene import BuildingScene
from gibson2.core.render.viewer import Viewer
import pybullet as p


def main():
    global _mouse_ix, _mouse_iy, down, view_direction

    ###################################################################################
    # load the scene to get traversable area (disable the whole area for performance) #
    ###################################################################################
    p.connect(p.GUI)
    p.setGravity(0,0,-9.8)
    p.setTimeStep(1./240.)

    scene = BuildingScene('Allensville',
                          build_graph=True,
                          pybullet_load_texture=True)
    objects = scene.load()

    random_floor = scene.get_random_floor()
    p1 = scene.get_random_point_floor(random_floor)[1]
    p2 = scene.get_random_point_floor(random_floor)[1]
    shortest_path, geodesic_distance = scene.get_shortest_path(random_floor, p1[:2], p2[:2], entire_path=True)
    print('random point 1:', p1)
    print('random point 2:', p2)
    print('geodesic distance between p1 and p2', geodesic_distance)
    print('shortest path from p1 to p2:', shortest_path)

    p.disconnect()
    ###################################################################################


    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        model_path = os.path.join(get_model_path('Allensville'), 'mesh_z_up.obj')

    # set width, height (has to be equal for panorama)
    renderer = MeshRenderer(width=512, height=512)
    renderer.load_object(model_path)
    renderer.add_instance(0)

    print(renderer.visual_objects, renderer.instances)
    print(renderer.materials_mapping, renderer.mesh_materials)

    # set camera_pose / view direction
    camera_pose = np.array([0, 0, 1.2])
    view_direction = np.array([1, 0, 0])
    renderer.set_camera(camera_pose, camera_pose + view_direction, [0, 0, 1])
    renderer.set_fov(90)

    px = 0
    py = 0

    _mouse_ix, _mouse_iy = -1, -1
    down = False

    def change_dir(event, x, y, flags, param):
        global _mouse_ix, _mouse_iy, down, view_direction
        if event == cv2.EVENT_LBUTTONDOWN:
            _mouse_ix, _mouse_iy = x, y
            down = True
        if event == cv2.EVENT_MOUSEMOVE:
            if down:
                dx = (x - _mouse_ix) / 100.0
                dy = (y - _mouse_iy) / 100.0
                _mouse_ix = x
                _mouse_iy = y
                r1 = np.array([[np.cos(dy), 0, np.sin(dy)], [0, 1, 0], [-np.sin(dy), 0, np.cos(dy)]])
                r2 = np.array([[np.cos(-dx), -np.sin(-dx), 0], [np.sin(-dx), np.cos(-dx), 0], [0, 0, 1]])
                view_direction = r1.dot(r2).dot(view_direction)
        elif event == cv2.EVENT_LBUTTONUP:
            down = False

    cv2.namedWindow('test')
    cv2.setMouseCallback('test', change_dir)

    while True:
        with Profiler('Render'):
            frame = renderer.render(modes=('rgb', 'normal', '3d'))
        cv2.imshow('test', cv2.cvtColor(np.concatenate(frame, axis=1), cv2.COLOR_RGB2BGR))
        q = cv2.waitKey(1)
        if q == ord('w'):
            px += 0.05
        elif q == ord('s'):
            px -= 0.05
        elif q == ord('a'):
            py += 0.05
        elif q == ord('d'):
            py -= 0.05
        elif q == ord('q'):
            break
        camera_pose = np.array([px, py, 1.2])
        renderer.set_camera(camera_pose, camera_pose + view_direction, [0, 0, 1])

    renderer.release()


if __name__ == '__main__':
    main()