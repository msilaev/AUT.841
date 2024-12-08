#!/usr/bin/env python3

import rospy
import random
import moveit_commander
import moveit_msgs
import tf2_ros

import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

import sys
import rospy
import copy
import PyKDL 
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Pose
import tf
from geometry_msgs.msg import PoseStamped


from motion_test_pkg.srv import BoxSpawner, BoxSpawnerRequest, RandomPose
from motion_test_pkg.srv import BoxAttach, BoxAttachRequest
from moveit_commander import RobotCommander, MoveGroupCommander
import tf2_ros
import geometry_msgs.msg
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose


def is_pose_reachable(x, y, z, move_group, target_frame="right_gripper_base_link"):
    """
    Checks if a pose is reachable by the robot.

    Args:
        x (float): X-coordinate in the world frame.
        y (float): Y-coordinate in the world frame.
        z (float): Z-coordinate in the world frame.
        move_group (MoveGroupCommander): MoveIt group instance.
        target_frame (str): The frame to transform the pose into (default: "robot1_base_link").

    Returns:
        bool: True if the pose is reachable, False otherwise.
    """
    try:
        # Transform pose to the target frame
        tr_pose = transform_pose(x, y, z, target_frame=target_frame)
        if not tr_pose:
            rospy.logerr("Failed to transform pose")
            return False


        # Set roll, pitch, yaw angles (in radians)
        roll = 0*3.14  # 180 degrees
        pitch = 0.0  # No tilt
        yaw = 0.0    # No rotation around the Z-axis

# Convert to quaternion
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

# Set pose
        pose = Pose()
        pose.position.x = tr_pose[0]
        pose.position.y = tr_pose[1]
        pose.position.z = tr_pose[2]
        pose.orientation.x = 0
        pose.orientation.y = 1
        pose.orientation.z = 0
        pose.orientation.w = 0
        move_group.set_pose_target(pose)

        # Plan to the target and check if it's valid
        plan = move_group.plan()

        print("plan", plan[0])
        if plan[0] and len(plan.joint_trajectory.points) > 0:
            return True
        else:
            rospy.logwarn("Pose is not reachable")
            return False

    except Exception as e:
        rospy.logerr(f"Error in is_pose_reachable: {e}")
        return False


def transform_pose(x, y, z, target_frame="one_base_link"):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # Create PoseStamped in the source frame
        source_pose = geometry_msgs.msg.PoseStamped()
        source_pose.header.frame_id = "world"  # Replace with the source frame of `top_box`
        source_pose.pose.position.x = x
        source_pose.pose.position.y = y
        source_pose.pose.position.z = z

        # Use a timestamp slightly in the past
        source_pose.header.stamp = rospy.Time.now() - rospy.Duration(0.1)

        # Check if transform is available
        if not tf_buffer.can_transform("world", target_frame, source_pose.header.stamp, rospy.Duration(1.0)):
            rospy.logerr(f"Transform from 'world' to '{target_frame}' not available.")
            return None, None, None

        # Transform to target frame
        transformed_pose = tf_buffer.transform(source_pose, target_frame, rospy.Duration(1.0))
        return (
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z,
        )
    except Exception as e:
        rospy.logerr(f"TF transform failed: {e}")
        return None, None, None


#Functions for poses conversion
def frame_to_pose(frame):
	pose_result = Pose()
	pose_result.position.x = frame.p[0] 
	pose_result.position.y = frame.p[1] 
	pose_result.position.z = frame.p[2] 
	ang = frame.M.GetQuaternion() 
	pose_result.orientation.x = ang[0] 
	pose_result.orientation.y = ang[1] 
	pose_result.orientation.z = ang[2] 
	pose_result.orientation.w = ang[3]
	return pose_result

#Wait until a modification in the objects in the scene is updated
def wait_update_object(object_name, object_is_known=False, object_is_attached=False):
	global scene
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < 4) and not rospy.is_shutdown():
		# Test if the object is in attached objects
		attached_objects = scene.get_attached_objects([object_name])
		is_attached = len(attached_objects.keys()) > 0
		# Test if the object is in the scene. Note that attaching the object will remove it from known_objects
		is_known = object_name in scene.get_known_object_names()
		# Test if we are in the expected state
		if (object_is_attached == is_attached) and (object_is_known == is_known):
			return True
		# Sleep so that we give other threads time on the processor
		rospy.sleep(0.1)
		seconds = rospy.get_time()
	return False



def get_random_pose():
    """Call the random pose generator service and get random coordinates."""
    rospy.wait_for_service('/random_pose_generator')
    try:
        random_pose_service = rospy.ServiceProxy('/random_pose_generator', RandomPose)
        response = random_pose_service()
        return response.x, response.y, response.z
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None, None

def get_robot_pose(listener, robot_name):
    """Get the current pose of a robot."""
    try:
        now = rospy.Time(0)
        listener.waitForTransform("/world", robot_name, now, rospy.Duration(1.0))
        trans, rot = listener.lookupTransform("/world", robot_name, now)
        return trans
    except tf.Exception as e:
        rospy.logwarn(f"Could not get transform for {robot_name}: {e}")
        return None

def spawn_box(box_name, x, y, base):
    """
    Function to spawn a box by calling the /scene_spawner/spawn_box service.
    """
    rospy.wait_for_service('/scene_spawner/spawn_box')
    try:
        # Create a service proxy
        spawn_box_service = rospy.ServiceProxy('/scene_spawner/spawn_box', BoxSpawner)
        
        # Prepare the request
        req = BoxSpawnerRequest(name=box_name, x=x, y=y, base=base)
        
        # Call the service
        resp = spawn_box_service(req)
        
        # Check response
        if resp.success:
            rospy.loginfo(f"Successfully spawned box: {box_name}")
        else:
            rospy.logerr(f"Failed to spawn box: {resp.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node("motion_control_node")

    rospy.loginfo("Starting motion control node")

    # Wait for required services
    rospy.wait_for_service('/scene_spawner/spawn_box')
    rospy.wait_for_service('/scene_spawner/attach_release_box')

    # Service proxies
    spawn_box_service = rospy.ServiceProxy('/scene_spawner/spawn_box', BoxSpawner)
    attach_box_service = rospy.ServiceProxy('/scene_spawner/attach_release_box', BoxAttach)

  
    # Initialize MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm1 = moveit_commander.MoveGroupCommander("arm_one")
    gripper1 = moveit_commander.MoveGroupCommander("one_gripper")

    arm2 = moveit_commander.MoveGroupCommander("arm_two")
    gripper2 = moveit_commander.MoveGroupCommander("two_gripper")

    arm1.clear_pose_targets()
    gripper1.clear_pose_targets()

    arm2.clear_pose_targets()
    gripper2.clear_pose_targets()

    ###########

      # Spawn base box
    print("spawn")
    base_x, base_y, base_z = get_random_pose() 

    rospy.loginfo(f"Random pose for base box: x={base_x}, y={base_y}, z={base_z}")

    print((f"Random pose for base box: x={base_x}, y={base_y}, z={base_z}"))
    
    if base_x is not None:
        base_box_request = BoxSpawnerRequest(name="base_box", x=base_x, y=base_y, base=True)
        spawn_box_service(base_box_request)
        rospy.loginfo("Spawned base box")

    # Spawn top box
    #while True:
    #    top_x, top_y, top_z = get_random_pose()
    #    print(top_x, top_y, top_z, is_pose_reachable(top_x, top_y, top_z, arm1))
    #    if is_pose_reachable(top_x, top_y, top_z, arm1, "right_gripper_base_link"):
    #        break
    #    if is_pose_reachable(top_x, top_y, top_z, arm2, "left_gripper_base_link"):
    #        break
           
    
    top_x, top_y, top_z = get_random_pose()
    rospy.loginfo(f"Random pose for top box: x={top_x}, y={top_y}, z={top_z}")
    #print("Reachable arm1", is_pose_reachable(top_x, top_y, top_z))
    #print("Reachable arm2", is_pose_reachable(top_x, top_y, top_z, group_name="arm2"))

    top_x, top_y, top_z = get_random_pose()

    if top_x is not None:
        top_box_request = BoxSpawnerRequest(name="top_box", x=top_x, y=top_y, base=False)
        spawn_box_service(top_box_request)
        rospy.loginfo("Spawned top box")


    # Target pose for top box

    target_robot_base1 = transform_pose(top_x, top_y, top_z, target_frame="one_base_link")
    target_robot_base2 = transform_pose(top_x, top_y, top_z, target_frame="two_base_link")
    print("target pose1", target_robot_base1)
    
    object_frame = PyKDL.Frame()
    object_frame.p = PyKDL.Vector(target_robot_base1[0], target_robot_base1[1], target_robot_base1[2])
    object_frame.p = PyKDL.Vector(-0.6157501926826476, -0.13474749456119417, 0.0)
    #object_frame.p = PyKDL.Vector(-0.5, 0.55, 0.3)
    gripper_length = 0.24
    
    goal_frame = copy.deepcopy(object_frame)
    goal_frame.p[2] += gripper_length
    goal_frame.M.DoRotX(3.14)  # Gripper pointing down
    
    goal_pose = frame_to_pose(goal_frame)
    pose_stamped_target_wrist = geometry_msgs.msg.PoseStamped()
    pose_stamped_target_wrist.header.frame_id = "one_base_link"  # Adjust if needed
    pose_stamped_target_wrist.pose = goal_pose
    #Defines the offset pose
    pose_stamped_target_offset_wrist = copy.deepcopy(pose_stamped_target_wrist)
    pose_stamped_target_offset_wrist.pose.position.z += 0.1


    try:
        box_attach_request_0 = BoxAttachRequest(robot_name="robot1", box_name="top_box", attach=False)
        attach_box_service(box_attach_request_0)

        box_attach_request_01 = BoxAttachRequest(robot_name="robot1", box_name="base_box", attach=False)
        attach_box_service(box_attach_request_01)
    except:
        pass

    # Move to target pose
    rospy.loginfo("Moving to target pose")
    arm1.set_pose_target(pose_stamped_target_offset_wrist)
    arm1.go(wait=True)

    # Open gripper
    rospy.loginfo("Opening gripper")
    gripper1.set_named_target('gripper1 open')
    gripper1.go(wait=True)

    # Attach the top box
    rospy.loginfo("Attaching top box")
    box_attach_request = BoxAttachRequest(robot_name="robot1", box_name="top_box", attach=True)
    attach_box_service(box_attach_request)

    # Close gripper
    rospy.loginfo("Closing gripper")
    gripper1.set_named_target('gripper1 pinch')
    gripper1.go(wait=True)

    # Move to home position
    rospy.loginfo("Moving to home position")
    arm1.set_named_target('arm1 home')
    arm1.go(wait=True)

    rospy.loginfo("Motion control complete")



    







    
