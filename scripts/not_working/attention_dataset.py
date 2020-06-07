#!/usr/bin/env python2

import os
import shutil
import glob

import numpy as np
import torch
from torch.utils.data import Dataset

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation


category_ids = {
  'Keep' : '0',
  'Discard' : '1',
}

class AttentionDataset(Dataset):

  def __init__(self, path=None, transform=None, pre_transform=None):
    self.categories = ['Keep','Discard']
    self.transform = transform
    self.pre_transform = pre_transform
    self.data_list = [] 
    super(AttentionDataset, self).__init__()
    if path is not None: self.load_data(path)


  # accepts pointcloud as numpy array with each row as one point
  # fields are:
  #   0-2: xyz position of point
  #   3: point intensity
  #   4: point doppler
  #   5: point label
  def add_item(self, point_cloud):
    pos = torch.from_numpy(point_cloud[:, :3]).float()
    x = torch.from_numpy(point_cloud[:, 3:6]).float()
    y = torch.from_numpy(point_cloud[:, 6]).long()
    data = Data(pos=pos, x=x, y=y, category=y)
    data = data if self.pre_transform is None else self.pre_transform(data)
    self.data_list.append(data)

  # splits the dataset into train/test or train/val/test segments
  # accepts an array of doubles of length 2 or 3: [train, test, val]
  # entries must sum to 1.0
  def split(self, split):
    sum_split = 0
    for i in split: sum_split += i
    if sum_split != 1.0:
      print("entries in split must sum to 1.0")
      return

    n_examples = len(self.data_list)
    n_train = int(split[0] * n_examples)
    n_test = int(split[1] * n_examples)

    indices = np.arange(n_examples)
    np.random.shuffle(indices)
    train_indices = indices[:n_train]
    test_indices = indices[n_train:n_train+n_test]

    train_set = AttentionDataset(transform=self.transform, 
                                 pre_transform=self.pre_transform)
    train_set.data_list = [self.data_list[i] for i in train_indices]
    test_set = AttentionDataset(transform=None, 
                                pre_transform=self.pre_transform)
    test_set.data_list = [self.data_list[i] for i in test_indices]
    

    if len(split) == 3:
      n_val = int(split[2] * n_examples)
      val_indices = indices[n_train+n_test:n_train+n_test+n_val]
      val_set = AttentionDataset(transform=self.transform, 
                                 pre_transform=self.pre_transform)
      val_set.data_list = [self.data_list[i] for i in val_indices]
      return train_set,test_set,val_set

    return train_set,test_set

  def num_classes(self):
    return 2

  # save dataset into the specified file
  def save_data(self, filename):

    sum_nodes = 0
    for point_cloud in self.data_list:
      sum_nodes += point_cloud.pos.shape[0]

    arr = np.zeros([sum_nodes, 8])
    pcl_idx = 0
    i = 0
    for point_cloud in self.data_list:
      n_points = point_cloud.pos.shape[0]
      arr[i:i+n_points, :3] = point_cloud.pos.numpy()
      arr[i:i+n_points, 3:6] = point_cloud.x.numpy()
      arr[i:i+n_points, 6] = point_cloud.y.numpy()
      arr[i:i+n_points, -1] = pcl_idx
      i += n_points
      pcl_idx += 1

    np.save(filename, arr)

  # load dataset from file
  def load_data(self, filename):
    arr = np.load(filename)
    point_cloud = np.array([])
    cur_cloud = -1
    for i in range(arr.shape[0]):
      if cur_cloud != arr[i,-1]:
        if point_cloud.size != 0:
          self.add_item(point_cloud) 
        point_cloud = np.expand_dims(arr[i,:7],0)
        cur_cloud = arr[i,-1]
      else:
        point_cloud = np.concatenate((point_cloud,np.expand_dims(arr[i,:7],0)),0)
    self.add_item(point_cloud)


  def animate_points(self, iteration, ax):
    ax.clear()
    ax.set_xlim3d([-2,2])
    ax.set_ylim3d([-2,2])
    ax.set_zlim3d([-2,2])

    # assemble point sets in numpy arrays
    keep_points = np.array([])
    discard_points = np.array([])
    for i in range(self.data_list[iteration].pos.shape[0]):
      if self.data_list[iteration].y[i] == 0.0:
        if keep_points.size == 0:
          keep_points = np.expand_dims(self.data_list[iteration].pos[i,:],0)
        else:
          keep_points = np.concatenate((keep_points,
                                       np.expand_dims(self.data_list[iteration].pos[i,:],0)),
                                       0)
      else:
        if discard_points.size == 0:
          discard_points = np.expand_dims(self.data_list[iteration].pos[i,:],0)
        else:
          discard_points = np.concatenate((discard_points,
                                          np.expand_dims(self.data_list[iteration].pos[i,:],0)),
                                          0)

    # plot points
    ax.scatter(keep_points[:,0],
               keep_points[:,1],
               keep_points[:,2],
               c='g',s=5.0)
    ax.scatter(discard_points[:,0],
               discard_points[:,1],
               discard_points[:,2],
               c='r',s=5.0)
    

  # visualize dataset to verify it's been loaded correctly
  def visualize_data(self):
    # initialize figure
    fig = plt.figure()
    ax = Axes3D(fig)

    # assemble point sets in numpy arrays
    keep_points = np.array([])
    discard_points = np.array([])
    for i in range(self.data_list[0].pos.shape[0]):
      if self.data_list[0].y[i] == 0.0:
        if keep_points.size == 0:
          keep_points = np.expand_dims(self.data_list[0].pos[i,:],0)
        else:
          keep_points = np.concatenate((keep_points,
                                        np.expand_dims(self.data_list[0].pos[i,:],0)),
                                        0)
      else:
        if discard_points.size == 0:
          discard_points = np.expand_dims(self.data_list[0].pos[i,:],0)
        else:
          discard_points = np.concatenate((discard_points,
                                          np.expand_dims(self.data_list[0].pos[i,:],0)),
                                          0)

    ax.set_xlim3d([-20,20])
    ax.set_ylim3d([-20,20])
    ax.set_zlim3d([-10,10])

    # plot points
    ax.scatter(keep_points[:,0],
               keep_points[:,1],
               keep_points[:,2],
               c='g',s=5.0)
    ax.scatter(discard_points[:,0],
               discard_points[:,1],
               discard_points[:,2],
               c='r',s=5.0)
    
    i = len(self.data_list)

    ani = animation.FuncAnimation(fig, 
                                  self.animate_points, 
                                  i, 
                                  fargs=([ax]),
                                  interval=50,
                                  blit=False,
                                  repeat=True)

    plt.show()




  def __getitem__(self, indices):
    data = self.data_list.__class__()

    if torch.is_tensor(indices):
      indices = indices.tolist()

    data = self.data_list[indices]

    data = data if self.transform is None else self.transform(data)

    return data

  def __len__(self):
    return len(self.data_list)

