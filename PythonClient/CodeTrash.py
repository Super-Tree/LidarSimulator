da = frame
a = np.logical_and(np.logical_and(RSlidar32M_points[:, 1] == RSlidar32M_points[da, 1],
                                  RSlidar32M_points[:, 2] == RSlidar32M_points[da, 2]),
                   RSlidar32M_points[:, 0] == RSlidar32M_points[da, 0])
res = np.where(a)
for i in range(len(res[0])):
    print(RSlidar32M_points[res[0][i]])
