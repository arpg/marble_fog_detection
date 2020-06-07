import numpy as np
from data_loader import DataLoader, RadarPoint
from tqdm import tqdm

class CountGrid:
  def __init__(self, discretization, loader=None):
    self.discretization = discretization
    self.loader = loader
    if self.loader is not None:
      self.initialize_grids()
      self.accumulate_scans()


  def set_loader(self, loader):
    self.loader = loader
    self.initialize_grids()
    self.accumulate_scans()


  def get_loader(self):
    return self.loader


  def get_keys(self):
    return self.keys


  def get_limits(self):
    return self.min_x, self.max_x, self.min_y, self.max_y, self.min_z, self.max_z


  def get_x_limits(self):
    return self.min_x, self.max_x


  def get_y_limits(self):
    return self.min_y, self.max_y


  def get_z_limits(self):
    return self.min_z, self.max_z


  def get_discretization(self):
    return self.discretization


  def get_size(self):
    return self.num_x, self.num_y, self.num_z


  def get_cell(self, point, key, t_range=None):
    indices = self.point_to_idx(point)
    if indices is None:
      return None
    x_idx, y_idx, z_idx = indices
    result = self.count_grids[key][x_idx][y_idx][z_idx]
    if t_range is not None:
      result = [r for r in result if r[0] > t_range[0] and r[0] < t_range[1]]
    return result


  def accumulate_scans(self):
    print('accumulating scans')
    for scan in tqdm(self.loader):
      for key in self.keys:
        for point in scan[key]:
          indices = self.point_to_idx(point.point)
          if indices is not None:
            x_idx, y_idx, z_idx = indices
            self.count_grids[key][x_idx][y_idx][z_idx].append(
              (scan['timestamp'],point.intensity))


  def initialize_grids(self):
    print('initializing count grid')
    if len(self.loader) == 0:
      raise ValueError('cannot pass empty loader')
    self.keys = [s for s in self.loader[0].keys() if s[-1]=='w']
    self.get_bounds()
    self.x_offset = int(self.min_x/self.discretization)
    self.y_offset = int(self.min_y/self.discretization)
    self.z_offset = int(self.min_z/self.discretization)
    self.num_x = int(np.ceil((self.max_x-self.min_x)/self.discretization))
    self.num_y = int(np.ceil((self.max_y-self.min_y)/self.discretization))
    self.num_z = int(np.ceil((self.max_z-self.min_z)/self.discretization))
    self.count_grids = {}
    for key in self.keys:
      self.count_grids[key] = [[[[] for k in range(self.num_z)] 
                                      for j in range(self.num_y)] 
                                        for i in range(self.num_x)]

  def get_bounds(self):
    min_x = 0.0
    max_x = 0.0
    min_y = 0.0
    max_y = 0.0
    min_z = 0.0
    max_z = 0.0

    for scan in tqdm(self.loader):
      for hp in scan['radar_w']:
        point = hp.point[:3,:] / hp.point[3,0]
        if point[0,0] < min_x: min_x = point[0,0]
        if point[0,0] > max_x: max_x = point[0,0]
        if point[1,0] < min_y: min_y = point[1,0]
        if point[1,0] > max_y: max_y = point[1,0]
        if point[2,0] < min_z: min_z = point[2,0]
        if point[2,0] > max_z: max_z = point[2,0]

    self.min_x = min_x
    self.max_x = max_x
    self.min_y = min_y
    self.max_y = max_y
    self.min_z = min_z
    self.max_z = max_z


  def point_to_idx(self, point):
    if point.shape[0] == 4:
      point = point[:3,:] / point[3,0]
    x_idx = int(point[0,0]/self.discretization)-self.x_offset
    y_idx = int(point[1,0]/self.discretization)-self.y_offset
    z_idx = int(point[2,0]/self.discretization)-self.z_offset

    key = self.keys[0]
    if (x_idx < 0 or x_idx >= len(self.count_grids[key])
      or y_idx < 0 or y_idx >= len(self.count_grids[key][0])
      or z_idx < 0 or z_idx >= len(self.count_grids[key][0][0])):
      return None

    return x_idx, y_idx, z_idx