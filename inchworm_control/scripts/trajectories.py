#! /usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

filepath = '/home/dhruv/catkin_ws/src/inchworm_ros/inchworm_control/scripts/trajectory.npy'


def main():

  y_start = 0.1524
  y_end = 0.2286
  path = np.array(
    [
      np.array([y_start,
                0.]),
      np.array([y_start + ((y_end - y_start) / 2),
                0.12]),
      np.array([y_end,
                0.15])
      # np.array([0.2,
      #           0.1]),
      # np.array([0.1,
      #           0.]),
      # np.array([0.2,
      #           0.1]),
      # np.array([0.3,
      #           0.]),
      # np.array([0.2,
      #           0.1]),
      # np.array([0.3,
      #           0.]),
      # np.array([0.2,
      #           0.1])
    ]
  )

  path_x = [path[i][0] for i in range(np.shape(path)[0])]
  path_y = [path[i][1] for i in range(np.shape(path)[0])]

  curve = np.polyfit(path_x, path_y, deg=3)

  t = np.linspace(y_start, y_end)
  curve_path = np.poly1d(curve)

  trajectory = [
    np.array([0.,
              round(i,
                    3),
              round(curve_path(round(i,
                                     3)),
                    3)]) for i in t
  ]

  rev_trajectory = []
  jrow = np.shape(trajectory)[0] - 1
  for irow in range(np.shape(trajectory)[0]):
    rev_trajectory.append(trajectory[jrow])
    jrow -= 1

  trajectory.extend(rev_trajectory)
  np.save(filepath, trajectory)
  print trajectory

  path_x = [trajectory[i][1] for i in range(np.shape(trajectory)[0])]
  path_y = [trajectory[i][2] for i in range(np.shape(trajectory)[0])]

  fig = plt.figure()
  # ax = fig.add_subplot(111, projection='3d')
  # Axes3D.plot(path_x, path_y)
  plt.plot(path_x, path_y)
  plt.show()

  #   curve[0] * pow(t,
  #                  3) + curve[1] * pow(t,
  #                                      2) + curve[2] * pow(t,
  #                                                          3) + curve[3]
  # ]

  print curve_path
  # plt.plot(curve_path)
  # plt.show()

  return


if __name__ == '__main__':
  main()