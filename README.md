## AUT.841

## Step 1: Create Workspace
mkdir assignment_ws

cd assignment_ws

## Step 2: Download Motoman

git clone -b kinetic-devel https://github.com/ros-industrial/motoman.git src/motoman

rosdep update

rosdep install --from-paths src/ --ignore-src --rosdistro noetic

catkin_make

source devel/setup.bash

roslaunch motoman_sia20d_moveit_config demo.launch

## Step 3: Create Project Packages

cd src

catkin_create_pkg my_sia20d urdf

catkin_create_pkg wsg50_gripper urdf

## Copy the contents of corresponding folders (my_sia20d, wsg50_gripper) into the newly created packages here

## Step 4: Build and Launch Project

cd ..

catkin_make

source devel/setup.bash

roslaunch my_sia20d test_sia20d.launch

# Step 5: Launch robot with moveit config

cd ~/assignment_ws

source devel/setup.bash

roslaunch assignment_moveit demo.launch

roslaunch my_moveit demo.launch
