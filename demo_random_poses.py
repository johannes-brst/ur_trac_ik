import numpy as np
import time
import math
from tqdm import tqdm
import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
import csv

def get_iks(position, orientation):
    # Initialize kinematics for UR5 robot arm
    ur5_kin = ikfastpy.PyKinematics()
    n_joints = ur5_kin.getDOF()

    args = _transformation_to_ik_arguments(position, orientation)
    s = ur5_kin.inverse(args)
    n_solutions = int(len(s)/n_joints)
    s = np.asarray(s).reshape(n_solutions,n_joints)
    configs = s.tolist()
    #print("configs\n")
    #print(configs)
    return configs

def _transformation_to_ik_arguments(position, orientation):
    T = orientation.transformation_matrix
    T[:, -1][:-1] = position
    return T[:-1].reshape(-1)

joint_states = []
failed = 0

with open("solutions.csv", newline='') as csvfile:
    csvreader = csv.reader(csvfile, quoting = csv.QUOTE_NONNUMERIC, delimiter=',')
    for row in csvreader:
        joint_states.append(row)

'''for _ in tqdm(range(10000)):
    point = (np.random.random(3) - 0.5)
    iks = get_iks(point, Quaternion.random())
    #print("iks\n")
    #print(iks)
    if not iks:
        failed += 1
    joint_states.extend(iks)'''


joint_states = np.array(joint_states)
#print(joint_states);
plt.figure(1, clear=True)

for i in range(6):
    plt.subplot(2, 3, i + 1)
    plt.hist(joint_states[:, i])
    plt.title('joint {}'.format(i))
plt.show()





