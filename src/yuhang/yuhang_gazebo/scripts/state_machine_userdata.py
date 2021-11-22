#!/usr/bin/env python


import rospy
import smach
import smach_ros

from move_group_interface import MoveGroupInterface
from task_info import TaskInfo

robot_move = MoveGroupInterface()

task = TaskInfo()
'''
[
  (-0.05, 0.25), (-0.05, 0.30), (-0.05, 0.35),
  (-0.15, 0.45), (-0.15, 0.39), (-0.15, 0.33),
  (-0.25, 0.45),(-0.25, 0.39), (-0.25, 0.33)
]
'''
def print_cube_poses(poses):
    for i in range(len(poses)):
        print("cube[{}]: {}".format(i, poses[i]))

def robot_execution(target_label):
    rospy.loginfo("====EXECUTION STARTS")
    cube_id = task.get_cube_id(target_label)
    sub_id = task.get_sub_ind_by_label(target_label)
    rospy.loginfo("picking: \n\tcube ID: {}\n\tsub ID:{}\n\tcube label:{}".format(cube_id, sub_id, target_label))

    task.cube_poses = robot_move.cube_poses
    print_cube_poses(task.cube_poses)
    try_count = 0
    sub_changed = False

    
    total_try = 3
    # print(task.cube_poses)
    # print(cube_id)
    # print(task.get_cube_ind(cube_id))
    # print(task.cube_poses[task.get_cube_ind(cube_id)])
    while sub_changed != True:
        robot_move.isexecuting = True
        if try_count == total_try:
            break
        rospy.loginfo("try {} times".format(try_count+1))

        print("available cubes: total #[{}], {}".format(len(task.cube_poses[task.get_cube_ind(cube_id)]), task.cube_poses[task.get_cube_ind(cube_id)]))
        if task.cube_poses[task.get_cube_ind(cube_id)] != []:
            try_count += 1
            x,y = task.cube_poses[task.get_cube_ind(cube_id)].pop()
            # print("the object pose is : {},{}".format(
            #     x,y)
            # )
            robot_move.do_pick(x,y)
            robot_move.set_target(sub_id)
            # print("the target pose is : {},{}".format(
            #     robot_move.target_pose.position.x, robot_move.target_pose.position.y)
            # )
            robot_move.do_place()
            rospy.sleep(1)

            task.status[0] = robot_move.sub_status
            print("status goal:")
            print(task.goal_status[0])
            print("status current")
            print(task.status[0])
            sub_changed = task.check_sub_changed(task.status, sub_id)
            rospy.loginfo("after pick and place, sub changed: {}".format(sub_changed))
        rospy.sleep(0.5)
    # should be deleted task.cube_poses[task.cube_ind[target_label]] = task.cube_poses[task.cube_ind[target_label]][1:]
    task.move(target_label)
    print(task)
    robot_move.isexecuting = False
    print_cube_poses(task.cube_poses)

    rospy.loginfo("====EXECUTION ENDS")

    return sub_id

# static states
class Demo_a(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

        robot_move.go_pose("home")
        robot_move.gripper_open()
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Demo_a>>')
        task.cube_poses = robot_move.cube_poses

        sub_id = -1
        task.status[0] = robot_move.sub_status
        sub_changed = task.check_sub_changed(task.status, sub_id)
        while sub_changed == False:
            task.status[0] = robot_move.sub_status
            sub_changed = task.check_sub_changed(task.status, sub_id)
            if not isinstance(sub_changed, bool):
                rospy.loginfo("the sub changed: {}".format(sub_changed))
                task.set_sub_label('a', sub_changed)
                task.set_cube_id('a', robot_move.sub_target_id[sub_changed])
                break
            rospy.sleep(0.1)
            rospy.loginfo("sub didn't change!")
        
        task.cube_poses = task.cube_poses[1:]
        task.move('a')
        print(task)
        return 'out1'

class Demo_b(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['out1'],
            input_keys=['demo_sub_ind', 'demo_cube_id'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Demo_b>>')

        task.set_sub_label('b', userdata.demo_sub_ind)
        task.set_cube_id('b', userdata.demo_cube_id)
        task.move('b')

        print(task)
        return 'out1'

class Demo_c(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['out1'],
            input_keys=['demo_sub_ind', 'demo_cube_id'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Demo_c>>')

        task.set_sub_label('c', userdata.demo_sub_ind)
        task.set_cube_id('c', userdata.demo_cube_id)
        task.move('c')
        print(task)
        return 'out1'


# EXECUTING STATE s
class Move_a2(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['out1','out2'],
            output_keys=['demo_sub_ind', 'demo_cube_id'])

    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Move_a2>>')

        sub_id = robot_execution('a')
        tracking = robot_move.tracking_while_executing
        sub_changed = task.check_sub_changed(task.status, sub_id)
        rospy.loginfo("sub_changed: {}".format(sub_changed))
        if sub_changed != False:
            rospy.loginfo("demonstration: {}".format(sub_changed))
            demo = tracking.pop()
            userdata.demo_sub_ind = demo[0]
            userdata.demo_cube_id = demo[1]
            # print(tracking)
            return 'out2'
        else:
            rospy.loginfo("no more demonstration")
            return 'out1'


class Move_a3(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['out1','out2'],
            output_keys=['demo_sub_ind', 'demo_cube_id'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Move_a3>>')

        sub_id = robot_execution('a')
        tracking = robot_move.tracking_while_executing

        sub_changed = task.check_sub_changed(task.status, sub_id)
        print(sub_changed,tracking)
        if sub_changed != False:
            rospy.loginfo("demonstration: {}".format(sub_changed))
            demo = tracking.pop()
            userdata.demo_sub_ind = demo[0]
            userdata.demo_cube_id = demo[1]
            return 'out1'
        else:
            rospy.loginfo("no more demonstration")
            ordered_tuples = sorted(task.sub_labels.items(), key=lambda item: item[1])
            for i, label in ordered_tuples:
                if label != None:
                    cube_ind_tem = task.get_cube_ind(task.cube_ids[label])
                    if task.cube_poses[cube_ind_tem] != []:
                        # rospy.loginfo("{}[{}] is not none and {} is not empty".format(
                        #     label,
                        #     i,
                        #     task.cube_poses[cube_ind_tem]
                        # ))

                        return 'out2'
            robot_move.isexecuting = True
            task.status[0] = robot_move.sub_status
            sub_changed = task.check_sub_changed(task.status, sub_id)

            while sub_changed == False:
                print('sub not changed')
                task.status[0] = robot_move.sub_status
                sub_changed = task.check_sub_changed(task.status, -1)
                rospy.sleep(0.5)
            robot_move.isexecuting = False
            rospy.loginfo("demonstration: {}".format(sub_changed))
            print(sub_changed, robot_move.sub_target_id)
            userdata.demo_sub_ind = sub_changed
            userdata.demo_cube_id = robot_move.sub_target_id[sub_changed]
            return 'out1'


class Move_b2(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['out1','out2','out3'],
            output_keys=['demo_sub_ind', 'demo_cube_id'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Move_b2>>')

        sub_id = robot_execution('b')
        tracking = robot_move.tracking_while_executing

        sub_changed = task.check_sub_changed(task.status, sub_id)
        if sub_changed != False:
            rospy.loginfo("demonstration: {}".format(sub_changed))
            demo = tracking.pop()
            userdata.demo_sub_ind = demo[0]
            userdata.demo_cube_id = demo[1]
            return 'out2'
        else:
            rospy.loginfo("no more demonstration")

            ordered_tuples = sorted(task.sub_labels.items(), key=lambda item: item[1])
            for i, label in ordered_tuples:
                if label != None:
                    cube_ind_tem = task.get_cube_ind(task.cube_ids[label])
                    print("[error]: cube_poses[{}] in {}".format(cube_ind_tem, task.cube_poses))
                    if task.cube_poses[cube_ind_tem] != []:
                        # rospy.loginfo("{}[{}] is not none and {} is not empty".format(
                        #     label,
                        #     i,
                        #     task.cube_poses[cube_ind_tem]
                        # ))
                        if label == 'a':
                            return 'out3'
                        else:
                            return 'out1'

            task.status[0] = robot_move.sub_status
            sub_changed = task.check_sub_changed(task.status, sub_id)
            while sub_changed == False:
                task.status[0] = robot_move.sub_status
                sub_changed = task.check_sub_changed(task.status, sub_id)
                rospy.sleep(0.5)
            return 'out2'


class Move_b3(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['out1','out2'],
            output_keys=['demo_sub_ind', 'demo_cube_id'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Move_b3>>')

        sub_id = robot_execution('b')
        tracking = robot_move.tracking_while_executing

        sub_changed = task.check_sub_changed(task.status, sub_id)
        if sub_changed != False:
            rospy.loginfo("demonstration: {}".format(sub_changed))
            demo = tracking.pop()
            userdata.demo_sub_ind = demo[0]
            userdata.demo_cube_id = demo[1]
            return 'out1'
        else:
            rospy.loginfo("no more demonstration")
            ordered_tuples = sorted(task.sub_labels.items(), key=lambda item: item[1])
            for i, label in ordered_tuples:
                if label != None:
                    cube_ind_tem = task.get_cube_ind(task.cube_ids[label])
                    if task.cube_poses[cube_ind_tem] != []:
                        # rospy.loginfo("{}[{}] is not none and {} is not empty".format(
                        #     label,
                        #     i,
                        #     task.cube_poses[cube_ind_tem]
                        # ))

                        return 'out2'

            robot_move.isexecuting = True
            task.status[0] = robot_move.sub_status
            sub_changed = task.check_sub_changed(task.status, sub_id)
            tracking = robot_move.tracking_while_executing

            while sub_changed == False:
                print('sub not changed')
                task.status[0] = robot_move.sub_status
                sub_changed = task.check_sub_changed(task.status, -1)
                rospy.sleep(0.5)
            robot_move.isexecuting = False
                
            print(sub_changed, robot_move.sub_target_id)
            userdata.demo_sub_ind = sub_changed
            userdata.demo_cube_id = robot_move.sub_target_id[sub_changed]
            return 'out1'

class Move_c2(smach.State):
    def __init__(self):
        smach.State.__init__(self,
        outcomes=['out1','out2','out3'],
        output_keys=['demo_sub_ind', 'demo_cube_id'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Move_c2>>')

        sub_id = robot_execution('c')
        tracking = robot_move.tracking_while_executing

        sub_changed = task.check_sub_changed(task.status, sub_id)
        if sub_changed != False or tracking != []:
            rospy.loginfo("demonstration: {}".format(sub_changed))
            demo = tracking.pop()
            userdata.demo_sub_ind = demo[0]
            userdata.demo_cube_id = demo[1]
            return 'out1'
        else:
            rospy.loginfo("no more demonstration")
            ordered_tuples = sorted(task.sub_labels.items(), key=lambda item: item[1])
            for i, label in ordered_tuples:
                if label != None:
                    cube_ind_tem = task.get_cube_ind(task.cube_ids[label])
                    if task.cube_poses[cube_ind_tem] != []:
                        # rospy.loginfo("{}[{}] is not none and {} is not empty".format(
                        #     label,
                        #     i,
                        #     task.cube_poses[cube_ind_tem]
                        # ))

                        if label == 'a':
                            return 'out3'
                        else:
                            return 'out2'

            task.status[0] = robot_move.sub_status
            sub_changed = task.check_sub_changed(task.status, sub_id)
            while sub_changed == False:
                task.status[0] = robot_move.sub_status
                sub_changed = task.check_sub_changed(task.status, sub_id)
                rospy.sleep(0.5)
            userdata.demo_sub_ind = None
            userdata.demo_cube_id = None
            return 'out1'

class Move_c3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])
        pass
    def execute(self, userdata):
        rospy.loginfo('EXECUTING STATE <<Move_c3>>')
        robot_execution('c')

        return 'out1'



def main():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Halt'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Demo_a', Demo_a(),
                                transitions={'out1':'Move_a2'})
        smach.StateMachine.add('Move_a2', Move_a2(),
                                transitions={'out1':'Move_a3',
                                             'out2':'Demo_b'})
        smach.StateMachine.add('Move_a3', Move_a3(),
                                transitions={'out1':'Demo_b',
                                             'out2':'Move_b3'})
        smach.StateMachine.add('Demo_b', Demo_b(),
                                transitions={'out1':'Move_b2'})
        smach.StateMachine.add('Move_b2', Move_b2(),
                                transitions={'out1':'Move_b3',
                                             'out2':'Demo_c',
                                             'out3':'Move_a3'})
        smach.StateMachine.add('Move_b3', Move_b3(),
                                transitions={'out1':'Demo_c',
                                             'out2':'Move_c3'})
        smach.StateMachine.add('Demo_c', Demo_c(),
                                transitions={'out1':'Move_c2'})
        smach.StateMachine.add('Move_c2', Move_c2(),
                                transitions={'out1':'Move_c3',
                                             'out2':'Move_b3',
                                             'out3':'Move_a3'})
        smach.StateMachine.add('Move_c3', Move_c3(),
                                transitions={'out1':'Halt'})

    raw_input("--------------type 'start' to start the task-------------------------\n")
    # Execute SMACH plan
    outcome = sm.execute()

    robot_move.gripper_open()
    robot_move.go_pose("up")
    

if __name__ == '__main__':
    main()
