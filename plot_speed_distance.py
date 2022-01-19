import numpy as np
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import csv



joint_speeds = []
actual_qs = []
failed = 0

with open("newjointspeeds.csv", newline='') as csvfile:
    csvreader = csv.reader(csvfile, quoting = csv.QUOTE_NONNUMERIC, delimiter=',')
    for row in csvreader:
        joint_speeds.append(row)

with open("actual_q.csv", newline='') as csvfile:
    csvreader = csv.reader(csvfile, quoting = csv.QUOTE_NONNUMERIC, delimiter=',')
    for row in csvreader:
        actual_qs.append(row)

'''for _ in tqdm(range(10000)):
    point = (np.random.random(3) - 0.5)
    iks = get_iks(point, Quaternion.random())
    #print("iks\n")
    #print(iks)
    if not iks:
        failed += 1
    joint_states.extend(iks)'''


joint_speeds = np.absolute(np.array(joint_speeds))
actual_qs = np.absolute(np.array(actual_qs))
#print(joint_states);
plt.figure(1, clear=True)

"""for i in range(6):
    plt.subplot(2, 3, i + 1)
    plt.hist(joint_states[:, i])
    plt.title('joint {}'.format(i))
"""
plt.plot(joint_speeds, label = "joint_speed")
plt.plot(actual_qs, label = "actual_q")
plt.legend()
plt.grid()
plt.show()





