'''
  Uses PointNet architecture to classify radar points as keep or discard
  Ref "PointNet: Deep Learning on Point Sets for 3D Classification and Segmentation"
  Borrows heavily from the pointnet implementation at https://github.com/fxia22/pointnet.pytorch
'''

import numpy as np
import torch
import torch.nn.functional as F
import torch.autograd as autograd
from torch.nn import Linear as Lin, BatchNorm1d as BN, Conv1d


class Transform(torch.nn.Module):
  def __init__(self,k):
    super(Transform, self).__init__()
    self.k = k
    self.conv1 = Conv1d(self.k,64,1)
    self.conv2 = Conv1d(64,128,1)
    self.conv3 = Conv1d(128,1024,1)
    self.lin1 = Lin(1024,512)
    self.lin2 = Lin(512,256)
    self.lin3 = Lin(256,self.k**2)

    self.bn1 = BN(64)
    self.bn2 = BN(128)
    self.bn3 = BN(1024)
    self.bn4 = BN(512)
    self.bn5 = BN(256)

  def forward(self, x):
    batchsize = x.size()[0]
    x = F.relu(self.bn1(self.conv1(x)))
    x = F.relu(self.bn2(self.conv2(x)))
    x = F.relu(self.bn3(self.conv3(x)))
    x = torch.max(x,2,keepdim=True)[0]
    x = x.view(-1,1024)
    x = F.relu(self.bn4(self.lin1(x)))
    x = F.relu(self.bn5(self.lin2(x)))
    x = self.lin3(x)
    eye = autograd.Variable(torch.from_numpy(np.eye(self.k).flatten().astype(np.float32))).view(1,self.k**2).repeat(batchsize,1)
    if x.is_cuda:
      eye = eye.cuda()
    x = x + eye
    x = x.view(-1,self.k,self.k)
    return x


class PointNet(torch.nn.Module):
  def __init__(self, num_classes):
    super(PointNet, self).__init__()

    self.num_classes = num_classes

    # pointnet classification layers
    self.t1 = Transform(3)
    self.t2 = Transform(64)

    self.conv1 = Conv1d(6,64,1)
    self.conv2 = Conv1d(64,128,1)
    self.conv3 = Conv1d(128,1024,1)

    self.bn1 = BN(64)
    self.bn2 = BN(128)
    self.bn3 = BN(1024)

    # pointnet segmentation layers
    self.conv4 = Conv1d(1088,512,1)
    self.conv5 = Conv1d(512,256,1)
    self.conv6 = Conv1d(256,128,1)
    self.conv7 = Conv1d(128,num_classes,1)

    self.bn4 = BN(512)
    self.bn5 = BN(256)
    self.bn6 = BN(128)

  def forward(self,data):
    x = data['pos']
    num_points = x.size()[2]
    batch_size = x.size()[0]
   
    # input transform of position only
    input_tf = self.t1(x)
    x = x.transpose(2,1)
    x = torch.bmm(x,input_tf)
    x = x.transpose(2,1)
    
    # concatenate other features
    x = torch.cat((x,data['x']),1)
    
    # calculate point features
    x = F.relu(self.bn1(self.conv1(x)))
    
    feature_tf = self.t2(x)
    x = x.transpose(2,1)
    x = torch.bmm(x,feature_tf)
    x = x.transpose(2,1)
    
    #feature_tf = None

    point_feature = x

    # calculate global feature
    x = F.relu(self.bn2(self.conv2(x)))
    x = self.bn3(self.conv3(x))
    x = torch.max(x, 2, keepdim=True)[0]

    # concatenate global feature and point features
    x = x.view(-1,1024,1).repeat(1,1,num_points)
    x = torch.cat([x,point_feature],1)

    # get category scores
    x = F.relu(self.bn4(self.conv4(x)))
    x = F.relu(self.bn5(self.conv5(x)))
    x = F.relu(self.bn6(self.conv6(x)))
    x = self.conv7(x)
    x = x.transpose(2,1).contiguous()
    x = F.log_softmax(x.view(-1, self.num_classes), dim=-1)
    x = x.view(batch_size, num_points, self.num_classes)
    return x, input_tf, feature_tf

