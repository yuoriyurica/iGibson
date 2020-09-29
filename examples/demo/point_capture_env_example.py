from gibson2.envs.locomotor_env import TravEnv
import numpy as np
import gibson2
import os
import cv2

def main():
    config_filename = os.path.join(os.path.dirname(gibson2.__file__),
                                   '../examples/configs/point_capture_demo.yaml')

    nav_env = TravEnv(config_file=config_filename, mode='gui')

    for j in range(20):
        # reset robot position and random select a new one
        nav_env.reset()
        for i in range(1):
            # get current robot position
            pos = nav_env.get_position_of_interest()
            # get current robot orientation (in [x, y, z, w])
            orn = nav_env.get_orientation()
            print(pos, orn)
            # capture image and save
            to_image(nav_env.get_rgb(), os.path.join(os.path.dirname(gibson2.__file__), '../examples/captures', str(j) + '.png'))
            # set robot position and orientaiton
            nav_env.step((pos, orn))

def to_image(img, name):
    # crop the top half portion of image
    _img = cv2.cvtColor(img * 255, cv2.COLOR_RGB2BGR)[:256, :].astype(np.uint8)
    cv2.imwrite(name, _img)

if __name__ == "__main__":
    main()