#!/usr/bin/env python

import numpy as np



NUM_CUBES   = 3
NUM_TYPES   = 3
NUM_SUBS    = 3

'''
id: 201     202     203
ind:  0       1       2
'''
ID_TO_INDEX = {
    '201': 0,
    '202': 1,
    '203': 2
}

class TaskInfo:
    def __init__(self):
        self.status = np.array([[[0]*NUM_CUBES for i in range(NUM_SUBS)],
                                [[1]*NUM_CUBES for i in range(NUM_TYPES)]])
        self.goal_status = np.array([[[0]*NUM_CUBES for i in range(NUM_SUBS)],
                                     [[1]*NUM_CUBES for i in range(NUM_TYPES)]])
        self.sub_labels = {
            0: None,
            1: None,
            2: None
        }

        self.targets = []
        self.cube_poses = []

        self.cube_ind = {
            'a': None,
            'b': None,
            'c': None
        }

        self.cube_ids = {
            'a': None,
            'b': None,
            'c': None
        }

    def __str__(self):
        return (
            "========== goal\n"+
            str(self.goal_status)+
            "\n---------- current\n"+
            str(self.status)+
            "\n"+str(self.targets)+
            "\n"+str(self.sub_labels)+
            "\n"+str(self.cube_ids)+
            "\n=========="
        )

    def set_cube_ind(self, cube, id):
        self.cube_ind[cube] = ID_TO_INDEX[str(id)]
        return

    def get_cube_ind(self, id):
        return ID_TO_INDEX[str(id)]

    def set_object(self, t):
        self.targets.insert(0, t)
        return

    def set_cube_id(self, cube, id):
        self.cube_ids[cube] = id
        self.set_cube_ind(cube, id)
        return

    def get_cube_id(self, cube):
        return self.cube_ids[cube]

    def set_sub_label(self, label, sub_id):
        self.sub_labels[sub_id] = label
        return

    def get_sub_ind_by_label(self, label):
        for i in range(NUM_SUBS):
            if label == self.sub_labels[i]:
                return i
        return None
    def set_sub_labels(self, sub_ids):
        for i in range(len(sub_ids)):
            self.sub_labels[i] = self.get_key(self.cube_ids, sub_ids[i])
        return

    def move(self, cube):
        sub = self.get_sub_ind_by_label(cube) # subarea index by label (e.g. 'a')
        if sub == None: # if subarea does not contain any cube: do nothing and return back 
            return
        ind = sub

        sub_arr = list(self.goal_status[0][sub])
        sub_arr.pop()
        sub_arr.insert(0,1)
        self.goal_status[0][sub] = sub_arr

        arr = list(self.goal_status[1][ind])
        arr.pop()
        arr.insert(0,0)
        self.goal_status[1][ind] = arr

        
        return

    def check_sub_changed(self, status, sub_id):
        ans = False
        for i in range(NUM_SUBS):
            sum_goal = 0
            sum_status= 0
            sum_goal = sum(self.goal_status[0][i])
            sum_status = sum(status[0][i])
            # print(sum_status, sum_goal, i, sub_id)
            if sum_status > sum_goal:
                if i == sub_id:
                    ans = True
                    return ans
                else:
                    ans = i

        return ans

    def get_key(self, my_dict, val):
        for key, value in my_dict.items():
             if val == value:
                 return key
        return None
