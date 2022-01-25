import numpy as np
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import csv



actual_qs = []
failed = 0


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


actual_qs = np.array(actual_qs)
#print(joint_states);
plt.figure(1, clear=True)

"""for i in range(6):
    plt.subplot(2, 3, i + 1)
    plt.hist(joint_states[:, i])
    plt.title('joint {}'.format(i))
"""
plt.plot(actual_qs)
plt.legend(["joint 1", "joint 2", "joint 3", "joint 4", "joint 5", "joint 6"])
plt.grid()
plt.show()





