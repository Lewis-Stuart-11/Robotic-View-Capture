import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
import numpy as np

from sensor_msgs.msg import JointState

# Handles all communication with the virtual robot
class RobotControl(object):
    def __init__(self, num_move_attempts: int =3, attachment_name: str="tool0",
                        wait_time: float=0.75, planning_time: float=3.0, 
                        planning_algorithm: str=None):
        
        super(RobotControl, self).__init__()

        # Initialises python moveit and ros 
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur5_group', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # Sets the path planning algorithm, if given, otherwise the default is used
        if planning_algorithm:
            move_group.set_planner_id(planning_algorithm)

        # Sets the max planning time before timeing out
        move_group.set_planning_time(planning_time)

        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.waypoints = []

        self.scene = scene
        self.robot = robot

        self.attachment_name = attachment_name

        # The number of path planning attempts to make for each position
        self.num_move_attempts = num_move_attempts

        # Pause time after each command is issued
        self.wait_time = wait_time

        self.current_joint_info = None

        # Subscribes to the joint_states topic to get the current robot state
        rospy.Subscriber("joint_states", JointState, self.update_joint_state)

        rospy.sleep(wait_time)

    # Returns if ros is currently active
    def get_is_ros_active(self) -> bool:
        return rospy.is_shutdown()

    # Returns current robot position
    def get_current_position(self) -> Pose:
        return self.move_group.get_current_pose().pose.position

    # Returns current quartonian rotation
    def get_current_orientation(self) -> Quaternion:
        return self.move_group.get_current_pose().pose.orientation

    # Returns current eulur roll, pitch and yaw
    def get_current_rpy(self):
        return self.move_group.get_current_rpy()

    # Updates the current robot's joint states 
    def update_joint_state(self, data: dict):
        self.current_joint_info = data

    # Attempts to move robot to position
    def move_arm(self, position: Pose, reset_orientation: bool=True) -> bool:
        pose = Pose()

        pose.position = position

        if reset_orientation:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
    
        return self.execute_new_pose(pose)

    # Attempts to alter the current robot rotation
    def reorient_arm(self, quartonian: Quaternion) -> bool:
        pose = Pose()

        pose.position = self.get_current_position()

        pose.orientation = quartonian

        return self.execute_new_pose(pose)

    # Attempts to move the robot to a new position with a new rotation
    def move_and_orientate_arm(self, position: Point, rotation: Quaternion) -> bool:

        pose = Pose()

        pose.position = position

        pose.orientation = rotation

        return self.execute_new_pose(pose)

    # Attempts to execute a new pose
    def execute_new_pose(self, pose: Pose) -> bool:
        
        # Attempts to move the robot to the new pose after a set 
        # number of attempts
        for i in range(self.num_move_attempts):
            self.move_group.set_pose_target(pose)

            success = self.move_group.go(wait=True)

            rospy.sleep(self.wait_time)

            self.move_group.stop()

            self.move_group.clear_pose_targets()

            if success:
                return True

        return False

    # Adds a pose which is relative to the current robot pose to the position queue
    def add_relative_pose(self, point: Point):
        current_pose = self.move_group.get_current_pose().pose

        current_pose.position.x += point.x 
        current_pose.position.y += point.y 
        current_pose.position.z += point.z 

        self.waypoints.append(copy.deepcopy(current_pose))

    # Adds a new position to the current position queue
    def add_pose(self, position: Point):
        new_pose = Pose()
        
        new_pose.position = position

        self.waypoints.append(copy.deepcopy(new_pose))


    # Creates a path plan for each position in the position queue
    def execute_plan(self) -> bool:
        if len(self.waypoints) == 0:
            return False

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        self.waypoints, 0.01, 0.0)         

        self.move_group.execute(plan, wait=True)

        self.waypoints = []

        return True

    # Ensures that objects are correctly added to a scene in a given time
    def ensure_collision_update(self, obj_name: str, timeout: float=10) -> bool:
        start = rospy.get_time()
        current_time = rospy.get_time()

        while(current_time-start < timeout) and not rospy.is_shutdown():

            attached_objects = self.scene.get_attached_objects([obj_name])
            
            is_attached = len(attached_objects.keys()) > 0
            is_known = obj_name in self.scene.get_known_object_names()

            if is_attached and is_known:
                return True

            rospy.sleep(0.1)

            current_time = rospy.get_time() 

        return False
    
    # Adds an object to the scene based on its specific type
    def add_obj_to_scene(self, obj_type: str, obj_name: str, obj_position: Point, 
                         obj_size: list, attach: bool=False, file_name: str=None):
        
        # Creates object position in the scene
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = self.robot.get_planning_frame()
        obj_pose.pose.position = obj_position
        obj_pose.pose.orientation.w = 1.0
        
        # Adds specific object to the scene of a specific object type
        if obj_type == "box":
            self.scene.add_box(obj_name, obj_pose, obj_size)
        elif obj_type == "sphere":
            self.scene.add_sphere(obj_name, obj_pose, radius=obj_size[0])
        elif obj_type == "mesh":
            self.scene.add_mesh(obj_name, obj_position, file_name, size=obj_size)
        else:
            raise Exception("Object type " + obj_type + " is not currently supported")

        rospy.sleep(self.wait_time)

        # Attaches object to the robot's attachement
        if attach:
            touch_links = self.robot.get_link_names()
            self.scene.attach_box(self.attachment_name, obj_name, touch_links=touch_links)

    # Adds box to scene
    def add_box_to_scene(self, obj_name: str, obj_position: Point, 
                         obj_size: list, attach: bool=False):
        self.add_obj_to_scene("box", obj_name, obj_position, obj_size, attach=attach)

    # Adds sphere to scene
    def add_sphere_to_scene(self, obj_name: str, obj_position: Point, 
                            obj_radius:float, attach: bool=False):
        self.add_obj_to_scene("sphere", obj_name, obj_position, [obj_radius], attach=attach)

    # Adds mesh to scene
    def add_mesh_to_scene(self, obj_name: str, obj_position: Point, obj_size: list, 
                          mesh_file_name: str, attach: bool=False):
        self.add_obj_to_scene("sphere", obj_name, obj_position, obj_size, 
                              file_name=mesh_file_name, attach=attach)

    # Removes an object with a specified name from the scene
    def remove_object_from_scene(self, obj_name):
        self.scene.remove_world_object(obj_name)

        success = self.ensure_collision_update(obj_name)

        if not success:
            raise Exception("Could not remove object " + obj_name + " from scene")
    
    # Retrieves the current robot's transform from the world to attachement
    def get_total_transform(self) -> tuple([list, list]):
        num_tries = 3

        trans = None
        rot = None

        listener = tf.TransformListener()

        # Attempts a set number of times to retrieve the robot's transforms by querying the 
        # transform ros topic using the TF listener 
        for i in range(num_tries):
            try:
                (trans, rot) = listener.lookupTransform("/world", "/"+self.attachment_name, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(self.wait_time)
                continue
        
        if trans is None or rot is None:
            raise Exception("Unable to get transform for robot")
        
        return trans, rot
    
    # Retrieves the robot's joint information by querying the ROS joint state topic
    def get_current_joint_info(self):
        current_time = rospy.Time.now().to_sec()
        
        current_joint_state = None

        num_tries = 3

        # Attempts to retrieve the joint state information, each time checking that the time the 
        # joints were last updated is not earlier than the time the function was executed. 
        # This avoids outdated joint state information being returned
        for i in range(num_tries):
            joint_time = float(self.current_joint_info.header.stamp.to_time())

            if current_time - float(joint_time) > 0:
                rospy.sleep(self.wait_time)
            else:
                current_joint_state = self.current_joint_info
                break
        
        if current_joint_state is None:
            raise Exception("Unable to get joint state for robot")

        return current_joint_state
       