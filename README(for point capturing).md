### installation
```a
git clone /*this repo*/ --recursive
cd iGibson

conda create -n py3-igibson python=3.6 anaconda
source activate py3-igibson
pip install -e .
```

### usage
- run `examples/demo/point_capture_env_example.py` (recommended) or `examples/demo/point_capture_example.py`
- generate segmentation mesh from 3DSceneGraph and create a soft link named `mesh_z_up_seg.obj`
- occupies the rgb channel (have to run 2 times for rgb and segmentation)

### point_capture_env_example
- use `.yaml` config file to choose model
- `examples/configs/point_capture_demo.yaml`

```yaml
model_id: Allensville # target model name in gibson database
model_type: '' # '' for original rgb mesh, _seg for generated segmentation mesh
gravity: 0 # set to zero to make sure position and orientation are correct
panorama: true # render panorama
image_width: 512 # for panorama rendering, width and height must be the same
image_height: 512
vertical_fov: 90 # 90 fov for panorama rendering
```

- `gibson2/assets/models/turtlebot/turtlebot.urdf`

```urdf
<link name="eyes">
  <inertial>
  <mass value="0.001"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/></inertial>
</link>
<joint name="eye_joint" type="fixed">
<!--set camera offset in rpy / xyz-->
<origin rpy="0 0 0" xyz="0 0 1.2"/> 
<parent link="camera_depth_frame"/>
<child link="eyes"/>
</joint>
```

### point_capture_example
- more light weight but less powerful
- basic movements around scene (using WASD)