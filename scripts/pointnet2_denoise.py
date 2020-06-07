'''
  Uses pointnet++ architecture to classify radar points as keep or discard
  Ref "PointNet++: Deep Hierarchical Feature Learning on Point Sets in a Metric Space"
    by Charles R. Qi, Li Yi, Hai Su, and Leonidas J. Guibas
  Draws heavily on example code from pytorch_geometric by Matthias Fey, TU Dortmund
'''

import argparse
import torch
import torch.nn.functional as F
import torch_geometric.transforms as T
from torch_geometric.data import DataLoader
from torch.nn import Sequential as Seq, Linear as Lin, ReLU, BatchNorm1d as BN 
from torch_geometric.nn import knn_interpolate, PointConv, fps, radius, global_max_pool
from torch_geometric.utils import intersection_and_union as i_and_u 
from attention_dataset import AttentionDataset


class FPModule(torch.nn.Module):
  def __init__(self, k, nn):
    super(FPModule, self).__init__()
    self.k = k
    self.nn = nn

  def forward(self, x, pos, batch, x_skip, pos_skip, batch_skip):
    x = knn_interpolate(x, pos, pos_skip, batch, batch_skip, k=self.k)
    if x_skip is not None:
      x = torch.cat([x, x_skip], dim=1)
    x = self.nn(x)
    return x, pos_skip, batch_skip


class SAModule(torch.nn.Module):
  def __init__(self, ratio, r, nn):
    super(SAModule, self).__init__()
    self.ratio = ratio
    self.r = r
    self.conv = PointConv(nn)

  def forward(self, x, pos, batch):
    idx = fps(pos, batch, ratio=self.ratio)
    row, col = radius(pos, pos[idx], self.r, batch, batch[idx],
      max_num_neighbors=64)
    edge_index = torch.stack([col, row], dim=0)
    x = self.conv(x, (pos, pos[idx]), edge_index)
    pos, batch = pos[idx], batch[idx]
    return x, pos, batch


class GlobalSAModule(torch.nn.Module):
  def __init__(self, nn):
    super(GlobalSAModule, self).__init__()
    self.nn = nn

  def forward(self, x, pos, batch):
    x = self.nn(torch.cat([x, pos], dim=1))
    x = global_max_pool(x, batch)
    pos = pos.new_zeros((x.size(0), 3))
    batch = torch.arange(x.size(0), device=batch.device)
    return x, pos, batch


def MLP(channels, batch_norm=True):
  return Seq(*[Seq(Lin(channels[i-1], channels[i]), ReLU(), BN(channels[i]))
    for i in range(1, len(channels))
  ])


# currently having problems with layer dims
# example implementation assumes data.x has dim 3 for the point normal
# but my data.x has dim 2 for doppler and intensity
class Net(torch.nn.Module):
  def __init__(self, num_classes):
    super(Net, self).__init__()
    self.sa1_module = SAModule(0.2, 0.2, MLP([3+3, 64, 64, 128]))
    self.sa2_module = SAModule(0.25, 0.4, MLP([128+3, 128, 128, 256]))
    self.sa3_module = GlobalSAModule(MLP([256+3, 256, 512, 1024]))

    self.fp3_module = FPModule(1, MLP([1024+256, 256, 256]))
    self.fp2_module = FPModule(3, MLP([256+128, 256, 128]))
    self.fp1_module = FPModule(3, MLP([128+3, 128, 128]))

    self.lin1 = torch.nn.Linear(128, 128)
    self.lin2 = torch.nn.Linear(128, 128)
    self.lin3 = torch.nn.Linear(128, num_classes)

  def forward(self, data):
    sa0_out = (data.x, data.pos, data.batch)
    sa1_out = self.sa1_module(*sa0_out)
    sa2_out = self.sa2_module(*sa1_out)
    sa3_out = self.sa3_module(*sa2_out)

    fp3_out = self.fp3_module(*sa3_out, *sa2_out)
    fp2_out = self.fp2_module(*fp3_out, *sa1_out)
    x, _, _ = self.fp1_module(*fp2_out, *sa0_out)

    x = F.relu(self.lin1(x))
    x = F.dropout(x, p=0.5, training=self.training)
    x = self.lin2(x)
    x = F.dropout(x, p=0.5, training=self.training)
    x = self.lin3(x)
    return F.log_softmax(x, dim=-1)


def train(model):
  model.train()

  total_loss = correct_nodes = total_nodes = 0
  for i, data in enumerate(train_loader):
    data = data.to(device)
    optimizer.zero_grad()
    out = model(data)
    loss = F.nll_loss(out, data.y)
    loss.backward()
    optimizer.step()
    total_loss += loss.item()
    correct_nodes += out.max(dim=1)[1].eq(data.y).sum().item()
    total_nodes += data.num_nodes

    if (i + 1) % 10 == 0:
      print('[{}/{}] Loss: {:.4f}, Train Accuracy: {:.4f}'.format(
        i + 1, len(train_loader), total_loss / 10,
        correct_nodes / total_nodes))
      total_loss = correct_nodes = total_nodes = 0

def test(loader,model):
  model.eval()

  correct_nodes = total_nodes = 0
  intersections, unions, categories = [], [], []
  for data in loader:
    data = data.to(device)
    with torch.no_grad():
      out = model(data)
    pred = out.max(dim=1)[1]
    correct_nodes += pred.eq(data.y).sum().item()
    total_nodes += data.num_nodes
    i, u = i_and_u(pred, data.y, test_dataset.num_classes(), data.batch)
    intersections.append(i.to(torch.device('cpu')))
    unions.append(u.to(torch.device('cpu')))
    categories.append(data.category.to(torch.device('cpu')))

  category = torch.cat(categories, dim=0)
  intersection = torch.cat(intersections, dim=0)
  union = torch.cat(unions, dim=0)
  ious = [[] for _ in range(len(loader.dataset.categories))]
  for j in range(len(loader.dataset)):
    i = intersection[j, category[j]]
    u = union[j, category[j]]
    iou = i.to(torch.float) / u.to(torch.float)
    iou[torch.isnan(iou)] = 1
    ious[category[j]].append(iou.mean().item())

  for cat in range(len(loader.dataset.categories)):
    ious[cat] = torch.tensor(ious[cat]).mean().item()

  return correct_nodes / total_nodes, torch.tensor(ious).mean().item()

if __name__=="__main__":

  parser = argparse.ArgumentParser(description='get datafile name')
  parser.add_argument('-f','--filename', type=str, help='filename from which to load training and test data')
  args = parser.parse_args()

  transform = T.Compose([
    T.RandomTranslate(0.01),
    T.RandomRotate(15, axis=0),
    T.RandomRotate(15, axis=1),
    T.RandomRotate(15, axis=2)
  ])
  pre_transform = T.NormalizeScale()

  dataset = AttentionDataset(path=args.filename,
                             transform=transform,
                             pre_transform=pre_transform)
  print("num scans in dataset: " + str(len(dataset)))
  split = [0.8,0.2]
  train_dataset,test_dataset = dataset.split(split)
  train_loader = DataLoader(train_dataset, 
                            batch_size=12, 
                            shuffle=True, 
                            num_workers=6)
  test_loader = DataLoader(test_dataset,
                           batch_size=12,
                           shuffle=False,
                           num_workers=6)

  device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
  model = Net(train_dataset.num_classes()).to(device)
  optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

  for epoch in (1, 100):
    train(model)
    acc, iou = test(test_loader,model)
    print('Epoch: {:02d}, Acc: {:.4f}, IoU: {:.4f}'.format(epoch, acc, iou))