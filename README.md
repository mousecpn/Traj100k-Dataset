# Traj100k-Dataset
The simulation dataset for Robot Trajectron

### Environment

 - pybullet
 - pybullet_data
 - numpy
 - scipy==1.10.1
 - pickle==1.23.5
 - roboticstoolbox-python: https://github.com/petercorke/robotics-toolbox-python.git

### Usage
```
python franka_6dof_grasp.py
```
You can set ```p.connect(p.DIRECT)``` to turn off the GUI.

Traj20k (``traj_fre20_noisy_20000.json``) is a mini-version of Traj100k, which contains the following attributes:

 - **ee_log**: Trajectories represented by Cartesian position of the end-effector (Main)
 - **joint_pos_log**: Trajectories represented by joint position of the arm
 - **joint_vel_log**: Velocity information of the trajectories
 - **frequency**: sampling frequency