#!/usr/bin/env python2

import numpy as np
import scipy.spatial.transform as tf
import rosbag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from tqdm import tqdm
import random

class RadarPoint:
  def __init__(self, point, intensity, doppler, distance):
    self.point = point
    self.intensity = intensity
    self.doppler = doppler
    self.distance = distance
    self.keep = False

class DataLoader:

  def __init__(self, bag_filename, radar_topic, 
               odom_topic=None, lidar_topic=None, 
               radar_tf=np.eye(4), lidar_tf=np.eye(4)):
    self.radar_topic = radar_topic
    self.odom_topic = odom_topic
    self.lidar_topic = lidar_topic
    self.T_rl = lidar_tf # transfrom from lidar to radar frame
    self.T_ro = radar_tf # transform from radar to odom frame
    bag_shortname = bag_filename.split('/')[-1]
    print('loading raw data from bag: ' + bag_shortname)
    raw_data = self.read_bag(bag_filename)
    print('temporally syncing messages')
    self.temporal_sync(raw_data)
    print('aligning pointclouds to world frame')
    self.spatial_align(raw_data)

  def get_points(self, scan_idx):
    radar_scan = self.scans[scan_idx]['radar_w']
    radar_points = radar_scan[0].point[:3,:]
    for i in range(1,len(radar_scan)):
      radar_points = np.concatenate((radar_points,radar_scan[i].point[:3,:]),1)

    lidar_points = None
    if self.lidar_topic is not None:
      lidar_scan = self.scans[scan_idx]['lidar_w']
      lidar_points = lidar_scan[0].point[:3,:]
      for i in range(1,len(lidar_scan)):
        if i % 2 == 0:
          lidar_points = np.concatenate((lidar_points,lidar_scan[i].point[:3,:]),1)

    return lidar_points, radar_points

  def animate(self, iteration):
    self.ax.clear()
    self.ax.set_xlim3d([-20,20])
    self.ax.set_ylim3d([-20,20])
    self.ax.set_zlim3d([-10,10])
    start_idx = max(0, iteration-3)

    lidar_points, radar_points = self.get_points(start_idx)
    
    for i in range(start_idx+1,iteration):
      next_lidar, next_radar = self.get_points(i)
      radar_points = np.concatenate((radar_points, next_radar), 1)
    
    if lidar_points is not None and lidar_points.shape[1] > 0:
      self.ax.scatter(lidar_points[0,:],
                      lidar_points[1,:],
                      lidar_points[2,:],
                      c='g', s=0.1)

    if radar_points.shape[1] > 0:
      self.ax.scatter(radar_points[0,:],
                      radar_points[1,:],
                      radar_points[2,:],
                      c='r', s=1.0)

    positions = np.expand_dims(self.scans[0]['odom'][:3,3],1)
    for i in range(1,iteration):
      next_pos = np.expand_dims(self.scans[i]['odom'][:3,3],1)
      positions = np.concatenate((positions,next_pos),1)
    self.ax.scatter(positions[0,:],
                    positions[1,:],
                    positions[2,:],
                    c='b', s=5.0)


  def display_scans(self):

    self.fig = plt.figure()
    self.ax = Axes3D(self.fig)
    
    iterations = len(self.scans)

    self.ax.set_xlim3d([-20,20])
    self.ax.set_ylim3d([-20,20])
    self.ax.set_zlim3d([-10,10])
    self.ax.view_init(25,10)

    lidar_points, radar_points = self.get_points(0)

    if lidar_points is not None and lidar_points.shape[1] > 0:
      self.ax.scatter(lidar_points[0,:],
                      lidar_points[1,:],
                      lidar_points[2,:],
                      c='g', s=0.5)

    if radar_points.shape[1] > 0:
      self.ax.scatter(radar_points[0,:],
                      radar_points[1,:],
                      radar_points[2,:],
                      c='r', s=2.0)

    self.ax.scatter(self.scans[0]['odom'][0,3],
                    self.scans[0]['odom'][1,3],
                    self.scans[0]['odom'][2,3],
                    c='b', s=5.0)

    ani = animation.FuncAnimation(self.fig, 
                                  self.animate, 
                                  iterations, 
                                  interval=100,
                                  blit=False,
                                  repeat=True)

    plt.show()


  def transform_pcl(self,cloud_s,T_WS):
    cloud_w = []
    for point in cloud_s:
      point_s = point.point
      point_w = T_WS.dot(point_s)
      cloud_w.append(RadarPoint(point_w,
                                point.intensity,
                                point.doppler,
                                point.distance))

    return cloud_w;


  def spatial_align(self,synced_msgs):
    self.scans = []
    num_scans = len(synced_msgs['radar'])
    for i in tqdm(range(num_scans)):

      # add minimal fields: timestamp sensor frame radar scan
      scan_entry = {'timestamp':synced_msgs['radar'][0][0],
                    'radar_s':synced_msgs['radar'][0][1]}

      # add radar pointcloud and odom message in world frame if available
      if self.odom_topic is not None:
        scan_entry['odom'] = synced_msgs['odom'][0][1]
        scan_entry['radar_w'] = self.transform_pcl(synced_msgs['radar'][0][1],
                                                         synced_msgs['odom'][0][1])

      # add lidar pointcloud in world frame if available
      if self.odom_topic is not None and self.lidar_topic is not None:
        scan_entry['lidar_w'] = self.transform_pcl(synced_msgs['lidar'][0][1],
                                                         synced_msgs['odom'][0][1])
      # add lidar pointcloud in radar sensor frame if no odom is available
      elif self.lidar_topic is not None:
        scan_entry['lidar_s'] = synced_msgs['lidar'][0][1]

      self.scans.append(scan_entry)
      
      del synced_msgs['radar'][0]
      if self.odom_topic is not None: del synced_msgs['odom'][0]
      if self.lidar_topic is not None: del synced_msgs['lidar'][0]
      


  # iterates through a set of raw messages
  # interpolates odom and lidar messages (if present) 
  # so all messages are temporally synced to the radar messages
  def temporal_sync(self, msgs):
    # sync odom msgs to radar msgs if available
    if self.odom_topic is not None:
      self.sync(msgs,'radar','odom')
    # sync lidar msgs to radar msgs if available
    if self.lidar_topic is not None:
      self.sync(msgs,'radar','lidar')

    # if both lidar and odom msgs are present, 
    # lidar sync may have truncated radar msgs 
    # beyond time range of the odom msgs
    # need to ensure all timestamps still align
    if self.odom_topic is not None and self.lidar_topic is not None:
      while len(msgs['odom']) > 0 and msgs['odom'][0][0] < msgs['radar'][0][0]:
        msgs['odom'].pop(0)
      while len(msgs['odom']) > 0 and msgs['odom'][-1][0] > msgs['radar'][-1][0]:
        msgs['odom'].pop(-1)

    # verify number of messages in each list matches and 
    # the start and end timestamps for each list are equal
    if self.odom_topic is not None:
      if len(msgs['odom']) != len(msgs['radar']):
        raise ValueError('Number of odom messages does not match number of radar messages')
      if msgs['odom'][0][0] != msgs['radar'][0][0]:
        raise ValueErrorError('Start timestamp of odom messages (' + str(msgs['odom'][0][0]) + 
          ') does not match start timestamp of radar messages (' + 
          str(msgs['radar'][0][0]) + ')')
      if msgs['odom'][-1][0] != msgs['radar'][-1][0]:
        raise ValueErrorError('Start timestamp of odom messages (' + str(msgs['odom'][-1][0]) + 
          ') does not match start timestamp of radar messages (' + 
          str(msgs['radar'][-1][0]) + ')')
    if self.lidar_topic is not None:
      if len(msgs['lidar']) != len(msgs['radar']):
        raise ValueErrorError('Number of lidar messages does not match number of radar messages')
      if msgs['lidar'][0][0] != msgs['radar'][0][0]:
        raise ValueErrorError('Start timestamp of lidar messages (' + str(msgs['lidar'][0][0]) + 
          ') does not match start timestamp of radar messages (' + 
          str(msgs['radar'][0][0]) + ')')
      if msgs['lidar'][-1][0] != msgs['radar'][-1][0]:
        raise ValueErrorError('Start timestamp of lidar messages (' + str(msgs['lidar'][-1][0]) + 
          ') does not match start timestamp of radar messages (' + 
          str(msgs['radar'][-1][0]) + ')')
 

  # accepts a dict of message lists,
  # a reference key (the list of messages to which the other messages will be synced)
  # and a sync key (the list of messages to be synced to the reference messages)
  def sync(self, msgs, ref_key, sync_key):
    ref_idx = 0 
    sync_idx = 0
    sync_msgs = msgs[sync_key]
    ref_msgs = msgs[ref_key]
    ref_msgs_out = []
    sync_msgs_out = []

    # ensure first odom message precedes first pcl message
    while (sync_msgs[sync_idx][0] >= ref_msgs[ref_idx][0]):
      ref_idx += 1

    # iterate over all reference messages
    while ref_idx < len(ref_msgs):
      # find sync messages immediately before and after
      # the current reference message
      while (sync_idx < len(sync_msgs) and 
        sync_msgs[sync_idx][0] < ref_msgs[ref_idx][0]):
        sync_idx += 1

      if (sync_idx >= len(sync_msgs)):
        break

      t_ref = ref_msgs[ref_idx][0]
      t_pre = sync_msgs[sync_idx-1][0]
      t_post = sync_msgs[sync_idx][0]

      c = (t_ref - t_pre) / (t_post - t_pre)

      if sync_key == 'odom':
        T_interp = self.interpolate_odom(sync_msgs[sync_idx-1], 
                                         sync_msgs[sync_idx], 
                                         c)
        sync_msgs_out.append((t_ref,T_interp))
      else:
        cloud_interp = self.interpolate_lidar(sync_msgs[sync_idx-1], 
                                              sync_msgs[sync_idx], 
                                              c)
        sync_msgs_out.append((t_ref,cloud_interp))

      ref_msgs_out.append(ref_msgs[ref_idx])
      ref_idx += 1

    msgs[ref_key] = ref_msgs_out
    msgs[sync_key] = sync_msgs_out


  def interpolate_odom(self, msg0, msg1, c):
    r_pre = msg0[1]
    r_post = msg1[1]
    q_pre = msg0[2]
    q_post = msg1[2]

    # interpolate pose at ref timestamp
    r_interp = (1.0 - c) * r_pre + c * r_post
    slerp = tf.Slerp([0.0,1.0],tf.Rotation([q_pre.as_quat(),q_post.as_quat()]))
    q_interp = slerp([c])[0]

    C_interp = q_interp.as_dcm()
    T_interp = np.identity(4)
    T_interp[0:3,0:3] = C_interp;
    T_interp[0:4,3] = r_interp.squeeze();

    return T_interp


  # depends on point i in cloud0 having the same sensor frame
  # azimuth and elevation angle as point i in cloud1
  # need to verify this is the case for the particular sensor
  def interpolate_lidar(self, msg0, msg1, c):
    cloud_interp = []
    for point0, point1 in zip(msg0[1],msg1[1]):
      '''
      point_interp = np.ones((4,1))
      point_interp[:3,:] = 0.5 * (point0[:3,:] + point1[:3,:])
      cloud_interp.append(point_interp)
      '''
      cloud_interp.append(point0 if c < 0.5 else point1)
    return cloud_interp


  # reads messages in topic_list from the given rosbag file
  # returns unsynced lists of messages from each topic
  def read_bag(self, bag_filename):
    
    print("HI")
    
    bag = rosbag.Bag(bag_filename)

    # iterate through bag file, storing relevant topics
    msgs = {'radar':[]}
    if self.odom_topic is not None:
      msgs['odom'] = []
    if self.lidar_topic is not None:
      msgs['lidar'] = []

    bag_msgs = bag.read_messages(topics=[self.radar_topic, 
                                         self.odom_topic, 
                                         self.lidar_topic])

    # for (topic, msg, t) in bag_msgs:
    #   print(topic, msg, t)

    print("self.radar_topic: ", self.radar_topic)
    print("self.odom_topic: ", self.odom_topic)
    print("self.lidar_topic: ", self.lidar_topic)

    # Will run out of memory with larger ROS bags! 
    # msg_list = [(topic, msg, t) for (topic, msg, t) in bag_msgs]

    time_start = bag_msgs(msg)

    # for (topic, msg, t) in bag_msgs:
    #   msg_list = (topic, msg, t)

    for (topic, msg, t) in tqdm(msg_list):
      if topic == self.radar_topic:
        gen = pcl.read_points(msg, 
                              skip_nans=True, 
                              field_names=('x','y','z','intensity','doppler','range'))
        cloud = []
        for entry in gen:
          point = np.array([[entry[0]],
                            [entry[1]],
                            [entry[2]],
                            [1.0]])
          point = np.matmul(self.T_ro,point)
          if np.linalg.norm(point[0:3]) > 1.0:
            cloud.append(RadarPoint(point,entry[3],entry[4],entry[5]))
        msgs['radar'].append((msg.header.stamp.to_sec(), cloud))

      elif self.odom_topic is not None and topic == self.odom_topic:
        r = np.array([[msg.pose.pose.position.x],
                      [msg.pose.pose.position.y],
                      [msg.pose.pose.position.z],
                      [1.0]])

        q_arr = np.array([msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w])

        if np.any(np.isnan(r)) or np.any(np.isnan(q_arr)): continue

        q = tf.Rotation.from_quat(q_arr)
        msgs['odom'].append((msg.header.stamp.to_sec(), r, q))

      elif self.lidar_topic is not None and topic == self.lidar_topic:
        gen = pcl.read_points(msg, 
                              skip_nans=True, 
                              field_names=('x','y','z'))
        cloud = []
        for entry in gen:
          if random.randint(0,4) == 0:
            point = np.array([[entry[0]],
                              [entry[1]],
                              [entry[2]],
                              [1.0]])
            point = np.matmul(self.T_ro,np.matmul(self.T_rl,point))
            if np.linalg.norm(point[0:3]) > 1.0:
              cloud.append(RadarPoint(point,0,0,0))
        msgs['lidar'].append((msg.header.stamp.to_sec(), cloud))

    bag.close()

    # ensure odom and pcl messages are sorted with increasing timestamp   
    msgs['radar'] = sorted(msgs['radar'])
    if self.odom_topic is not None: msgs['odom'] = sorted(msgs['odom'])
    if self.lidar_topic is not None: msgs['lidar'] = sorted(msgs['lidar'])

    return msgs


  # returns number of runs in the dataset
  def __len__(self):
    return len(self.scans)

  # return item at idx
  def __getitem__(self, idx):
    return self.scans[idx]

  # returns list of keys for each entry in the scan list
  def get_keys(self):
    try:
      return list(scans[0])
    except IndexError as e:
      print('list uninitialized, raised error: {0}'.format(e))

