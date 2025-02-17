## AUT.841 Assignment 1,  Part 1

### Step 1: Create Workspace
```bash
mkdir assignment_ws

cd assignment_ws
```

### Step 2: Download Motoman
```bash
git clone -b kinetic-devel https://github.com/ros-industrial/motoman.git src/motoman

rosdep update

rosdep install --from-paths src/ --ignore-src --rosdistro noetic

catkin_make

source devel/setup.bash

roslaunch motoman_sia20d_moveit_config demo.launch
```

### Step 3: Create Project Packages
```bash
cd src

catkin_create_pkg my_sia20d urdf

catkin_create_pkg wsg50_gripper urdf
```

### Copy the contents of corresponding folders (my_sia20d, wsg50_gripper) into the newly created packages here

### Step 4: Build and Launch Project
```bash
cd ..

catkin_make

source devel/setup.bash

roslaunch my_sia20d test_sia20d.launch
```

### Step 5: Launch robot with moveit config
```bash
cd ~/assignment_ws

source devel/setup.bash

roslaunch my_moveit2 demo.launch
```


## AUT.841 Assignment 1,  Part 2

### Step 1, 2 same as above

### Step 1: Create Workspace
```bash
mkdir assignment_ws

cd assignment_ws
```

### Step 2: Download Motoman
```bash
git clone -b kinetic-devel https://github.com/ros-industrial/motoman.git src/motoman

rosdep update

rosdep install --from-paths src/ --ignore-src --rosdistro noetic
```

### Step 3 

Copy files from Part2, together with CMakeText and package.xmp files in folder to 
src/

```bash
chmod +x src/motion_test_pkg/src/*.py

catkin_make

source devel/setup.bash
```

### Step 4

Open several terminals and source workspace. Then implement following commands in separate terminal each

#### Starting nodes 

```bash
roslaunch motion_test_pkg combined_action.launch
```
#### Publishing a Goal Message

To publish a goal message to the `/process_action/goal` topic, use the following command:

```bash
rostopic pub /process_action/goal motion_test_pkg/MoveRobotActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command: 'start'"
```

#### Showing feedback and result messages from the action server

```bash
rostopic echo /process_action/feedback
```

```bash
rostopic echo /process_action/result
```

### Step 5

Continue to debug, understand how to make robot arm move to box


