#!/usr/bin/env python





import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import tf

from geometry_msgs.msg import Point, Pose, PoseArray
from std_msgs.msg import Int32MultiArray
from setup_object import SetupObjects

GRIPPER_OPEN = 0.15
GRIPPER_CLOSED = 0.363
GRIPPER_CLOSED = 0.4 # new

pick_pose = Pose()
pick_pose.position.x = 0
pick_pose.position.y = 0.25
pick_pose.position.z = 1.19



pick_pose.orientation.x = -0.985644240594
pick_pose.orientation.y = -0.0378743138791
pick_pose.orientation.z = -0.0187875927762
pick_pose.orientation.w = 0.163456396906

class MoveGroupInterface(object):
    def __init__(self):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('Move_Group_Interface', anonymous=True)
        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()


        group_name = "arm" # See .srdf file to get available group names
        group = moveit_commander.MoveGroupCommander(group_name)
        hand_name = "hand" # See .srdf file to get available group names
        hand = moveit_commander.MoveGroupCommander(hand_name)
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)

        group.set_max_acceleration_scaling_factor(1)
        group.set_max_velocity_scaling_factor(1)
        group.set_goal_orientation_tolerance(3.14 / 40)
        group.set_goal_position_tolerance(0.02)
        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.hand = hand
        self.display_trajectory_publisher = display_trajectory_publisher


        pose = [-0.25, 0.5, 1.002, 0, 0, 0]
        self.object_pose = self.get_pose(pose)

        self.tracking = True



        self.gripper_control(GRIPPER_OPEN)
        self.go_pose("up")
        setup_obj = SetupObjects()



        self.last_x = 0
        self.last_y = 0



        pose = [0.25, 0.5, 1.002, 0, 0, 0]
        self.target_pose = self.get_pose(pose)



        self.listen_target_pose = True
        self.listen_sub_status_state = False



        # tracking data
        self.available_sub_poses = [[] for i in range(3)]
        self.cube_poses = [[] for i in range(3)]
        self.sub_status = np.array([[0]*3 for i in range(3)])
        self.sub_target_id = [-1]*3
        self.tracking_while_executing = []
        self.isexecuting = False
        # ==

        rospy.loginfo("WAITING FOR TOPICS...")
        rospy.wait_for_message('/tracking_object/available_sub', PoseArray)
        rospy.loginfo("/tracking_object/available_sub comes!")
        self.available_sub_sub = rospy.Subscriber(
        '/tracking_object/available_sub',
        PoseArray,
        self.sub_target_sub_callback)

        rospy.wait_for_message('/tracking_object/cube_201_poses', PoseArray)
        rospy.loginfo("/tracking_object/cube_201_poses comes!")
        self.cube_201_poses_sub = rospy.Subscriber(
        '/tracking_object/cube_201_poses',
        PoseArray,
        self.sub_cube_201_poses_callback)
        rospy.wait_for_message('/tracking_object/cube_202_poses', PoseArray)
        rospy.loginfo("/tracking_object/cube_202_poses comes!")
        self.cube_202_poses_sub = rospy.Subscriber(
        '/tracking_object/cube_202_poses',
        PoseArray,
        self.sub_cube_202_poses_callback)
        rospy.wait_for_message('/tracking_object/cube_203_poses', PoseArray)
        rospy.loginfo("/tracking_object/cube_203_poses comes!")
        self.cube_203_poses_sub = rospy.Subscriber(
        '/tracking_object/cube_203_poses',
        PoseArray,
        self.sub_cube_203_poses_callback)

        rospy.wait_for_message('/tracking_object/sub_status', Int32MultiArray)
        rospy.loginfo("/tracking_object/sub_status comes!")
        self.pos_status_sub = rospy.Subscriber(
        '/tracking_object/sub_status',
        Int32MultiArray,
        self.sub_status_callback)

        rospy.wait_for_message('/tracking_object/sub_target_id', Int32MultiArray)
        rospy.loginfo("/tracking_object/sub_target_id comes!")
        self.sub_target_id_sub = rospy.Subscriber(
        '/tracking_object/sub_target_id',
        Int32MultiArray,
        self.sub_target_id_callback)

        rospy.sleep(1)
        rospy.loginfo("ROBOT INITIALIZATION FINSHIED!")


    # ==========================================================================
    def plan_cartesian_path(self, scale=1, turns = []):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        current_pose = group.get_current_pose().pose
        #print("Current pose: ", current_pose)

        # Cartesian Paths
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        waypoints = []

        wpose = group.get_current_pose().pose

        for i in range(len(turns)):
            print("waypoint[{}]".format(i+1))
            # print(turns[i][0], turns[i][1])
            if turns[i][0] != []:
                if turns[i][0][0] != None:
                    wpose.position.x = turns[i][0][0]
                if turns[i][0][1] != None:
                    wpose.position.y = turns[i][0][1]
                if turns[i][0][2] != None:
                    wpose.position.z = turns[i][0][2]

            if turns[i][1] != []:

                if turns[i][1][0] != None:
                    wpose.orientation.x = turns[i][1][0]
                if turns[i][1][1] != None:
                    wpose.orientation.y = turns[i][1][1]
                if turns[i][1][2] != None:
                    wpose.orientation.z = turns[i][1][2]
                if turns[i][1][3] != None:
                    wpose.orientation.w = turns[i][1][3]

            waypoints.append(copy.deepcopy(wpose))


        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0,         # jump_threshold
                                           avoid_collisions = True)

        # Note: We are just planning, not asking move_group to actually move the robot yet:

        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # Displaying a Trajectory
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        #
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # Executing a Plan
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    # ==========================================================================







    # ==========================================================================
    # ##########################################################################
    # the pick operation
    def do_pick(self, x, y):
        t = []

        p = [x, y, None]
        o = []
        t.append([p,o])

        cartesian_plan, fraction = self.plan_cartesian_path(turns = t)
        rospy.sleep(0.5)
        self.display_trajectory(cartesian_plan)
        self.execute_plan(cartesian_plan)

        t = []
        p = [x, y, pick_pose.position.z]
        o = [-1,0,0,0]

        t.append([p,o])

        cartesian_plan, fraction = self.plan_cartesian_path(turns = t)
        rospy.sleep(0.5)
        self.display_trajectory(cartesian_plan)
        self.execute_plan(cartesian_plan)

        rospy.sleep(0.5)
        self.gripper_control(GRIPPER_CLOSED-0.001)

        rospy.sleep(0.25)

        self.gripper_control(GRIPPER_CLOSED)


        t = []
        p = [None, None, 1.4]
        o = []
        t.append([p,o])
        cartesian_plan, fraction = self.plan_cartesian_path(turns = t)
        self.execute_plan(cartesian_plan)
        return
    # ==========================================================================
    # ##########################################################################
    # the place operation
    def do_place(self):

        t = []

        p = [self.target_pose.position.x, self.target_pose.position.y, 1.5]
        o = []
        t.append([p,o])

        p = [self.target_pose.position.x, self.target_pose.position.y, pick_pose.position.z+0.001]
        o = [-1,0,0,0]

        t.append([p,o])
        cartesian_plan, fraction = self.plan_cartesian_path(turns = t)
        rospy.sleep(0.5)
        self.execute_plan(cartesian_plan)


        self.gripper_control(GRIPPER_OPEN)
        rospy.sleep(0.5)


        t = []
        p = [0, 0.45, 1.5]
        o = []
        t.append([p,o])
        cartesian_plan, fraction = self.plan_cartesian_path(turns = t)
        rospy.sleep(0.5)
        self.execute_plan(cartesian_plan)

        return

    def get_pose(self, pose):

        object_pose = Pose()
        object_pose.position.x = pose[0]
        object_pose.position.y = pose[1]
        object_pose.position.z = pose[2]

        quaternion = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

        object_pose.orientation.x = quaternion[0]
        object_pose.orientation.y = quaternion[1]
        object_pose.orientation.z = quaternion[2]
        object_pose.orientation.w = quaternion[3]

        return object_pose
    def go_pose(self, pose):
        try:
            arm = self.group
            arm.set_named_target(pose)
            arm.go(wait=True)
        except:
            pass
    # gripper controller
    def gripper_control(self, value):
        self.hand.set_joint_value_target([value]*6)
        return self.hand.go(wait=True)

    def gripper_open(self):
        self.gripper_control(GRIPPER_OPEN)
        return

    def gripper_close(self):
        self.gripper_control(GRIPPER_CLOSED)
        return

    def set_target(self, sub_id):
        if self.available_sub_poses[sub_id][0] != None:
            pose = self.available_sub_poses[sub_id][0]
            self.target_pose.position.x = pose[0]
            self.target_pose.position.y = pose[1]
            print("setting the object pose to {}, {}".format(pose[0], pose[1]))
        return


    def convert_poses(self, poses):
        a = [[] for i in range(3)]
        for i in range(len(poses)):
            index = -1

            if poses[i].position.y < 0.33:
                index = 0
            elif poses[i].position.y < 0.42:
                index = 1
            else:
                index = 2
            a[index].append( (poses[i].position.x, poses[i].position.y) )
        return a


    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # totpic callbacks
    # --------------------------------------------------------------------------
    def sub_target_sub_callback(self, poseArray):
        self.available_sub_poses = self.convert_poses(poseArray.poses)
        return

    def sub_cube_201_poses_callback(self, poseArray):
        self.cube_poses[0] = []
        for each in poseArray.poses:
            x, y = each.position.x, each.position.y
            self.cube_poses[0].append((x,y))
        return

    def sub_cube_202_poses_callback(self, poseArray):
        self.cube_poses[1] = []
        for each in poseArray.poses:
            x, y = each.position.x, each.position.y
            self.cube_poses[1].append((x,y))
        return

    def sub_cube_203_poses_callback(self, poseArray):
        self.cube_poses[2] = []
        for each in poseArray.poses:
            x, y = each.position.x, each.position.y
            self.cube_poses[2].append((x,y))
        return

    def sub_status_callback(self, msg):
        new_sub = np.array(msg.data).reshape(3,3)
        if self.isexecuting:
            for i in range(len(new_sub)):
                if sum(new_sub[i]) > sum(self.sub_status[i]) and sum(new_sub[i]) == 1:
                    while self.sub_target_id[i] == -1:
                        rospy.sleep(0.5)
                    if self.sub_target_id[i] not in self.tracking_while_executing:
                        self.tracking_while_executing.insert(
                            0, (i, self.sub_target_id[i])
                        )
        self.sub_status = np.array(msg.data).reshape(3,3)

        return

    def sub_target_id_callback(self, msg):
        self.sub_target_id = msg.data
        return




def main():
    try:
        move_group = MoveGroupInterface()
        move_group.go_pose('home')
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        print("STOP RUNNING")
        tutorial.group.stop()
        tutorial.group.clear_pose_targets()
        rospy.on_shutdown()
    return
if __name__ == '__main__':
  main()
