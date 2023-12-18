from matplotlib import pyplot as plt
from TrajectoryGenerator import MinimumTrajPlanner, getCircle, getLine
import numpy as np

zero = np.array([[0], [0], [0]])
# planner = MinimumTrajPlanner(getLine(20, 1, 0.1, 0, 0, 0, 'x')[:3], 1, 100, zero, zero, zero, zero, 4)
# planner = MinimumTrajPlanner(getCircle(5, 1, 10, 0, 0, 0)[:3], 3, 100, zero, zero, zero, zero, 3)
arr = np.array([np.linspace(0, 0, 100), np.linspace(0, 100, 100), np.linspace(0, 0, 100)])
planner = MinimumTrajPlanner(arr, 8, 100, zero, zero, zero, zero, 4)

traj = planner.computeTraj()
# np.savetxt("data.csv", traj[3], delimiter=',')
print(traj)
x = np.linspace(0, np.size(traj[4]), np.size(traj[4]))
y = traj[4]
x1 = np.linspace(0, np.size(traj[1]), np.size(traj[1]))
y1 = traj[1]
plt.figure()
plt.plot(x, y)
plt.plot(x1, y1)

plt.show()