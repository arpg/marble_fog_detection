import argparse
import torch
from pointnet_denoise import PointNet
import torch_geometric.transforms as T
import torch.nn.functional as F
from torch.utils.data import DataLoader
from attention_dataset import AttentionDataset
import numpy as np
from tqdm import tqdm

def feature_transform_regularizer(trans):
  d = trans.size()[1]
  batch_size = trans.size()[0]
  eye = torch.eye(d)[None,:,:]
  if trans.is_cuda:
    eye = eye.to('cuda')
  loss = torch.mean(torch.norm(torch.bmm(trans,trans.transpose(2,1))-eye, dim=(1,2)))
  return loss


def get_acc_ignore_padding(pred, target, is_padding):
  correct_nodes = total_nodes = 0
  for i in range(target.size()[0]):
    if not is_padding[i]:
      total_nodes += 1
      if pred[i] == target[i]:
        correct_nodes += 1
  return correct_nodes, total_nodes

# collate data list into batch
def collate(data_list):
  pos_list = [data.pos.transpose(0,1) for data in data_list]
  x_list = [data.x.transpose(0,1) for data in data_list]
  y_list = [data.y for data in data_list]
  padding = [[False]*data.y.size()[0] for data in data_list]

  # find max number of points in batch
  num_points = max([pos.shape[1] for pos in pos_list])

  # pad tensors with randomly sampled repeated points to match max number of points
  for i in range(len(pos_list)):
    point_diff = num_points - pos_list[i].shape[1]
    if point_diff > 0:
      idx = np.random.choice(np.arange(pos_list[i].shape[1]),point_diff)
      pos_list[i] = torch.cat((pos_list[i],pos_list[i][:,idx]),1)
      x_list[i] = torch.cat((x_list[i],x_list[i][:,idx]),1)
      y_list[i] = torch.cat((y_list[i],y_list[i][idx]),0)
      padding[i] += [True]*point_diff
    pos_list[i] = pos_list[i].unsqueeze(0)
    x_list[i] = x_list[i].unsqueeze(0)
    y_list[i] = y_list[i].unsqueeze(0)

  # stack tensors into 3d tensors of dim batch_size x num_points x n
  pos = torch.cat(pos_list,0)
  x = torch.cat(x_list,0)
  y = torch.cat(y_list,0)
  padding = torch.tensor(padding)

  return {'pos':pos,'x':x,'y':y,'padding':padding}



parser = argparse.ArgumentParser(description='get datafile name')
parser.add_argument('-f','--filenames', type=str, help='text file with filenames from which to load training and test data')
parser.add_argument('-s','--save_filename', type=str, help='file in which to save the weights of the trained model')
args = parser.parse_args()

file_list = []
dataset_file = open(args.filenames)
for file in dataset_file:
  file_list.append(file.strip())

transform = T.Compose([
  T.RandomRotate(45, axis=0),
  T.RandomRotate(45, axis=1),
  T.RandomRotate(45, axis=2)
])
pre_transform = T.NormalizeScale()

dataset = AttentionDataset(path=None,
                           transform=transform,
                           pre_transform=pre_transform)

for file in file_list:
  dataset.load_data(file)

print("num scans in dataset: " + str(len(dataset)))
split = [0.8,0.2]
train_dataset,test_dataset = dataset.split(split)
train_loader = DataLoader(train_dataset, 
                          batch_size=32, 
                          shuffle=True, 
                          num_workers=6,
                          drop_last=True,
                          collate_fn=collate)
test_loader = DataLoader(test_dataset,
                         batch_size=32,
                         shuffle=False,
                         num_workers=6,
                         collate_fn=collate)

num_classes = 2
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = PointNet(num_classes)
optimizer = torch.optim.Adam(model.parameters(), lr=0.001, betas=(0.9,0.999))
scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.5)
model.to(device)


for epoch in range(20):

  for i, data in enumerate(train_loader,0):

    data['pos'] = data['pos'].to(device)
    data['x'] = data['x'].to(device)
    data['y'] = data['y'].to(device)
    data['padding'] = data['padding'].to(device)

    # evaluate model
    optimizer.zero_grad()
    model = model.train()
    pred, input_tf, feature_tf = model(data)
    pred = pred.view(-1, num_classes)
    target = data['y'].view(-1,1)[:,0]

    # calculate loss
    loss = F.nll_loss(pred, target)
    #loss += feature_transform_regularizer(feature_tf) * 0.001

    # backprop and optimize weights
    loss.backward()
    optimizer.step()

    # evaluate
    pred_choice = pred.data.max(1)[1]
    correct_nodes = pred_choice.eq(target.data).cpu().sum()
    total_nodes = pred_choice.size()[0]

    print('[%d: %d/%d] Train loss: %f, Train Accuracy: %f' %
      (epoch+1, i + 1, len(train_loader), loss.item(),
      float(correct_nodes)/float(total_nodes)))
    
    if i % 10 == 0:
      j, data = next(enumerate(test_loader,0))
      data['pos'] = data['pos'].to(device)
      data['x'] = data['x'].to(device)
      data['y'] = data['y'].to(device)
      data['padding'] = data['padding'].to(device)

      model = model.eval()
      with torch.no_grad():
        pred, _, _ = model(data)
      pred = pred.view(-1, num_classes)
      target = data['y'].view(-1,1)[:,0]
      loss = F.nll_loss(pred,target)
      pred_choice = pred.data.max(1)[1]
      correct_nodes = pred_choice.eq(target.data).cpu().sum()
      total_nodes = pred_choice.size()[0]
      print('[%d: %d/%d] %s loss: %f, Accuracy: %f' %
      (epoch+1, i + 1, len(train_loader), 'Test', loss.item(),
      float(correct_nodes.item())/float(total_nodes)))

  optimizer.step()
  scheduler.step()
  
shape_ious = []
for i, data in tqdm(enumerate(test_loader,0)):
  data['pos'] = data['pos'].to(device)
  data['x'] = data['x'].to(device)
  data['y'] = data['y'].to(device)
  data['padding'] = data['padding'].to(device)

  model = model.eval()
  
  with torch.no_grad():
    pred, _, _ = model(data)
  pred_choice = pred.data.max(2)[1]

  pred_np = pred_choice.cpu().data.numpy()
  target_np = data['y'].cpu().data.numpy() 

  for shape_idx in range(target_np.shape[0]):
    parts = range(num_classes)
    part_ious = []
    for part in parts:
      I = np.sum(np.logical_and(pred_np[shape_idx] == part, target_np[shape_idx] == part))
      U = np.sum(np.logical_or(pred_np[shape_idx] == part, target_np[shape_idx] == part))
      if U == 0:
        iou = 1
      else:
        iou = I / float(U)
      part_ious.append(iou)
    shape_ious.append(np.mean(part_ious))

print('IoU for class {}: {}'.format('Chair', np.mean(shape_ious)))

torch.save(model.state_dict(),args.save_filename)