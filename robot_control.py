import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs
import numpy as np

class RobotControl(object):
    def __init__(self, num_move_attempts = 3, attachment_name="tool0",
                        wait_time=1):
        super(RobotControl, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur5_group', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        self.global_scale = 1
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.waypoints = []

        self.scene = scene
        self.robot = robot

        self.attachment_name = attachment_name
        self.num_move_attempts = num_move_attempts
        self.wait_time = wait_time

        #rospy.init_node("ur5_tf_listener")

        rospy.sleep(wait_time)

    def get_is_ros_active(self):
        return rospy.is_shutdown()

    def get_current_position(self):
        return self.move_group.get_current_pose().pose.position

    def get_current_orientation(self):
        return self.move_group.get_current_pose().pose.orientation

    def get_current_rpy(self):
        return self.move_group.get_current_rpy()

    def set_planning_time(self, planning_time):
        self.move_group.set_planning_time(planning_time)

    def move_arm(self, point_x, point_y, point_z, reset_orientation=True):
        pose = geometry_msgs.msg.Pose()

        pose.position.x = point_x * self.global_scale
        pose.position.y = point_y * self.global_scale
        pose.position.z = point_z * self.global_scale

        if reset_orientation:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0

            pose.orientation.w = 1.0
    
        return self.execute_new_pose(pose)


    def reorient_arm(self, rotate_x, rotate_y, rotate_z, rotate_w):
        pose = geometry_msgs.msg.Pose()

        pose.position = self.get_current_position()

        pose.orientation.x = rotate_x
        pose.orientation.y = rotate_y
        pose.orientation.z = rotate_z

        pose.orientation.w = rotate_w

        return self.execute_new_pose(pose)

    def move_and_orientate_arm(self, point_x, point_y, point_z, rotate_x, 
                                     rotate_y, rotate_z, rotate_w):
        pose = geometry_msgs.msg.Pose()

        pose.position.x = point_x * self.global_scale
        pose.position.y = point_y * self.global_scale
        pose.position.z = point_z * self.global_scale

        pose.orientation.x = rotate_x
        pose.orientation.y = rotate_y
        pose.orientation.z = rotate_z

        pose.orientation.w = rotate_w

        return self.execute_new_pose(pose)

    def execute_new_pose(self, pose):

        for i in range(self.num_move_attempts):
            self.move_group.set_pose_target(pose)

            success = self.move_group.go(wait=True)

            rospy.sleep(self.wait_time)

            self.move_group.stop()

            self.move_group.clear_pose_targets()

            if success:
                return True

        return False

    def add_relative_pose(self, point_x=0, point_y=0, point_z=0):
        current_pose = self.move_group.get_current_pose().pose

        current_pose.position.x += point_x * self.global_scale 
        current_pose.position.y += point_y * self.global_scale 
        current_pose.position.z += point_z * self.global_scale 

        self.waypoints.append(copy.deepcopy(current_pose))


    def add_pose(self, point_x, point_y, point_z):
        new_pose = geometry_msgs.msg.Pose()
        
        new_pose.position.x = point_x * self.global_scale 
        new_pose.position.y = point_y * self.global_scale 
        new_pose.position.z = point_z * self.global_scale 

        self.waypoints.append(copy.deepcopy(new_pose))


    def execute_plan(self):
        if len(self.waypoints) == 0:
            return False

        (plan, fraction) = self.move_group.compute_cartesian_path(
                                        self.waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        self.move_group.execute(plan, wait=True)

        self.waypoints = []

        return True


    def ensure_collision_update(self, obj_name, timeout=10):
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

    def add_box_to_scene(self, obj_name, obj_position, obj_size, attach=False):

        obj_pose = geometry_msgs.msg.PoseStamped()
        obj_pose.header.frame_id = self.robot.get_planning_frame()
        obj_pose.pose.position = obj_position
        obj_pose.pose.orientation.w = 1.0
        
        self.scene.add_box(obj_name, obj_pose, obj_size)

        #success = self.ensure_collision_update(obj_name)

        #if not success:
        #    raise Exception("Could not add object " + obj_name + " to scene")

        rospy.sleep(self.wait_time)

        if attach:
            touch_links = self.robot.get_link_names()
            self.scene.attach_box(self.attachment_name, obj_name, touch_links=touch_links)
    

    def add_sphere_to_scene(self, obj_name, obj_position, obj_radius, attach=False):

        obj_pose = geometry_msgs.msg.PoseStamped()
        obj_pose.header.frame_id = self.robot.get_planning_frame()
        obj_pose.pose.position = obj_position
        obj_pose.pose.orientation.w = 1.0

        self.scene.add_sphere(obj_name, obj_pose, radius=obj_radius)

        rospy.sleep(self.wait_time)

        if attach:
            touch_links = self.robot.get_link_names()
            self.scene.attach_box(self.attachment_name, obj_name, touch_links=touch_links)

    def add_mesh_to_scene(self, obj_name, obj_position, obj_size, mesh_file_name, attach=False):
        obj_pose = geometry_msgs.msg.PoseStamped()
        obj_pose.header.frame_id = self.robot.get_planning_frame()
        obj_pose.pose.position = obj_position
        obj_pose.pose.orientation.w = 1.0

        self.scene.add_mesh(obj_name, obj_position, mesh_file_name, size=obj_size)

        rospy.sleep(self.wait_time)

        if attach:
            touch_links = self.robot.get_link_names()
            self.scene.attach_box(self.attachment_name, obj_name, touch_links=touch_links)

    def remove_object_from_scene(self, obj_name):
        self.scene.remove_world_object(obj_name)

        success = self.ensure_collision_update(obj_name)

        if not success:
            raise Exception("Could not remove object " + obj_name + " from scene")
        
    def get_total_transform(self):
        num_tries = 3

        trans = None
        rot = None

        transformation_matrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        listener = tf.TransformListener()

        for i in range(num_tries):
            try:
                (trans, rot) = listener.lookupTransform("/world", "/tool0", rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(self.wait_time)
                continue
        
        if trans is None or rot is None:
            raise Exception("Unable to get transform for robot")
        
        return trans, rot

        """transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1

        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1

        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1

        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1
        transformation_matrix[0][0] = 1"""

        for i in trans:
            print i
        
        print "--------------------------"

        for i in rot:
            print i
        
    def get_current_joint_info(self):
        pass
       