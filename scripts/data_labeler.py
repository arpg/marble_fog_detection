#!/usr/bin/env python2

import sys
import argparse
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from attention_dataset import AttentionDataset
from data_loader import DataLoader, RadarPoint
from count_grid import CountGrid

import rosbag
import rospy

def get_keep_and_discard_sets(scan):
  keep_points = np.array([])
  discard_points = np.array([])
  lidar_points = np.array([])

  for point in scan['radar_w']:
    next_point = point.point[:3,:]
    if point.keep:
      if keep_points.size == 0:
        keep_points = next_point
      else:
        
        keep_points = np.concatenate((keep_points,next_point),1) 
    else:
      if discard_points.size == 0:
        discard_points = next_point
      else:
        discard_points = np.concatenate((discard_points,next_point),1) 

  for i,point in enumerate(scan['lidar_w']):
    if i%2 == 0:
      if lidar_points.size == 0:
        lidar_points = next_point
      else:
        next_point = point.point[:3,:]
        lidar_points = np.concatenate((lidar_points,next_point),1)

  return keep_points, discard_points, lidar_points


def animate_scans(iteration, loader, ax):
  ax.clear()
  ax.set_xlim3d([-20,20])
  ax.set_ylim3d([-20,20])
  ax.set_zlim3d([-10,10])
  start_idx = max(0, iteration-3)
  
  keep_points,discard_points,lidar_points = get_keep_and_discard_sets(loader[iteration])
  
  for i in range(start_idx+1,iteration):
    next_keep,next_discard,lidar_points = get_keep_and_discard_sets(loader[i])
    if next_keep.shape[0] > 0:
      if keep_points.shape[0] > 0:
        keep_points = np.concatenate((keep_points,next_keep),1)
      else:
        keep_points = next_keep
    if next_discard.shape[0] > 0:
      if discard_points.shape[0] > 0:
        discard_points = np.concatenate((discard_points,next_discard),1)
      else:
        discard_points = next_discard

  if keep_points.shape[0] > 0:
    ax.scatter(keep_points[0,:],
               keep_points[1,:],
               keep_points[2,:],
               c='g', s=2.0)

  if discard_points.shape[0] > 0:
    ax.scatter(discard_points[0,:],
               discard_points[1,:],
               discard_points[2,:],
               c='r', s=2.0)

  ax.scatter(lidar_points[0,:],
             lidar_points[1,:],
             lidar_points[2,:],
             c='b', s=0.1)
  
  positions = np.expand_dims(loader[0]['odom'][:3,3],1)

  for i in range(1,iteration):
    next_pos = np.expand_dims(loader[i]['odom'][:3,3],1)
    positions = np.concatenate((positions,next_pos),1)
  
  ax.scatter(positions[0,:],
             positions[1,:],
             positions[2,:],
             c='b', s=5.0)


def animate_pointclouds(loader):
  fig = plt.figure()
  ax = Axes3D(fig)
  
  iterations = len(loader)

  ax.set_xlim3d([-20,20])
  ax.set_ylim3d([-20,20])
  ax.set_zlim3d([-10,10])
  ax.view_init(25,10)
  
  keep_points, discard_points, lidar_points = get_keep_and_discard_sets(loader[0])

  if keep_points.shape[0] > 0:
    ax.scatter(keep_points[0,:],
               keep_points[1,:],
               keep_points[2,:],
               c='g', s=2.0)

  if discard_points.shape[0] > 0:
    ax.scatter(discard_points[0,:],
               discard_points[1,:],
               discard_points[2,:],
               c='r', s=0.5)

  ax.scatter(lidar_points[0,:],
             lidar_points[1,:],
             lidar_points[2,:],
             c='b', s=0.1)

  position = loader[0]['odom'][:3,:]
  ax.scatter(position[0],
             position[1],
             position[2],
             c='b', s=5.0)

  ani = animation.FuncAnimation(fig, 
                                animate_scans, 
                                iterations, 
                                fargs=(loader,ax),
                                interval=100,
                                blit=False,
                                repeat=False)

  plt.show()


def label_points(count_grid, t_range, threshold):

  for scan in count_grid.get_loader():
    timestamp = scan['timestamp']
    for hp_s,hp_w in zip(scan['radar_s'],scan['radar_w']):
      if 'lidar_w' not in count_grid.get_keys():
        time_range = [t + timestamp for t in t_range]
        cell_list = count_grid.get_cell(hp_w.point, 'radar_w', time_range)

        # sum intensities in cell that fall within specified time horizon
        sum_intensities = 0.0
        for entry in cell_list:
          if timestamp != entry[0]:
            sum_intensities += entry[1]
        if sum_intensities > threshold:
          hp_w.keep = True
          hp_s.keep = True
      else:
        cell_list = count_grid.get_cell(hp_w.point, 'lidar_w')
        if len(cell_list) > threshold:
          hp_w.keep = True
          hp_s.keep = True


# takes labeled points and loads them into a pytorch dataset object
def add_to_torch_dataset(loader, dataset):
  for scan in loader:
    cloud_arr = np.array([])
    for hp in scan['radar_s']:
      # de-homogenize point
      point = hp.point[:3,:] / hp.point[3,0]
      point_arr = np.array([[point[0,0],
                             point[1,0],
                             point[2,0],
                             hp.intensity,
                             hp.doppler,
                             hp.distance,
                             0.0]])
      if not hp.keep:
        point_arr[0,6] = 1.0

      if cloud_arr.size == 0:
        cloud_arr = point_arr
      else:
        cloud_arr = np.concatenate((cloud_arr,point_arr),0)

    dataset.add_item(cloud_arr)

def split_bag(bag_filename):
  bag = rosbag.Bag(bag_filename)

  start_time_s      = bag.get_start_time()
  end_time_s        = bag.get_end_time()
  duration_s        = end_time_s - start_time_s
  ros_start_time_s  = rospy.Time.from_sec(start_time_s)
  ros_end_time_s    = rospy.Time.from_sec(start_time_s + 10)
  # ros_end_time_s    = rospy.Time.from_sec(end_time_s)
  ros_duration_s    = ros_end_time_s - ros_start_time_s

  size_gb           = float(bag.size)/1028/1028/1028
  sec_per_gb        = duration_s / size_gb

  print 'start time [s]   : %f' % start_time_s
  print 'end time   [s]   : %f' % end_time_s
  print 'duration   [s]   : %f' % duration_s
  print 'size       [GB]  : %f' % size_gb
  print 'data rate  [s/GB]: %f' % sec_per_gb
  
  return ros_start_time_s, ros_end_time_s

if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='get bag filenames and directory')
  parser.add_argument('-d','--directory', type=str, help='root directory in which the bag files are stored')
  parser.add_argument('-b','--bags', type=str, help='text file containing bag file names')
  parser.add_argument('-r','--radar_topic', type=str, help='the raw radar pointcloud topic')
  parser.add_argument('-l','--lidar_topic', type=str, help='the raw lidar pointcloud topic')
  parser.add_argument('-o','--odom_topic', type=str, help='the odometry topic used to align pointclouds')
  parser.add_argument('-s','--save_filename', type=str, default='', help='filename to which the dataset will be saved')
  parser.add_argument('--voxel_len', type=float, default=0.1, help='edge length of voxels in the histogram')
  parser.add_argument('-t','--threshold', type=float, default=5.0, help='minimium summed intensity for voxel to be labeled as keep')
  parser.add_argument('--t_before', type=float, default=2.0, help='range of time in which past scans are considered')
  parser.add_argument('--t_after', type=float, default=0.0, help='range of time in which future scans are considered')

  parser.add_argument('-v','--visualizer', action='store_true', help='include -v to show visualizer')
  args = parser.parse_args()

  bag_name_list = []
  if args.directory[-1] != '/': args.directory = args.directory + '/'
  bag_list = open(args.directory + args.bags)
  for bag_name in bag_list:
    bag_name_list.append(args.directory + bag_name.strip())

  T_rl = np.array([[-1,0,0,0],[0,-1,0,0,],[0,0,1,0],[0,0,0,1]])
  #T_rl = np.eye(4)

  torch_dataset = AttentionDataset()
  for bag_filename in bag_name_list:

    ros_start_time_s, ros_end_time_s = split_bag(bag_filename)

    loader = DataLoader(bag_filename, 
                        args.radar_topic, 
                        args.odom_topic, 
                        args.lidar_topic,
                        lidar_tf=T_rl,
                        ros_start_time_s=ros_start_time_s,
                        ros_end_time_s=ros_end_time_s)

    if(args.visualizer):
      print("Starting to display scans ...")
      loader.display_scans()

    print("Starting to count grid ...")
    count_grid = CountGrid(args.voxel_len, loader)

    print("Starting to label points ...")
    t_range = [-args.t_before, args.t_after]
    label_points(count_grid, t_range, args.threshold)

    print("Starting to add to torch dataset ...")
    add_to_torch_dataset(loader, torch_dataset)

    if(args.visualizer):
      print("Starting to animate pointclouds ...")
      animate_pointclouds(loader)

    del loader
    del count_grid

  if args.save_filename != '':
    torch_dataset.save_data(args.save_filename)
