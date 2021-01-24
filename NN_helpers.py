from torch.utils.data import Dataset, DataLoader

# Load functions
import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.optim as optim
from torch.nn import Linear, GRU, Conv2d, Dropout2d, MaxPool2d, BatchNorm2d
from torch.nn.functional import relu, elu, relu6, sigmoid, tanh, softmax
from convlstm import *
import torch.utils.data
import numpy as np
import pandas as pd
import os
from ast import literal_eval
from actions import *


def transform(data, idx):
    t_q = torch.tensor(np.zeros([1,3,11,11]))
    for i in range(0,3):
        for (x,y) in data.query_state.loc[idx][i]:
            t_q[0,i,y+5,x+5]=1
    t_1 = torch.tensor(np.zeros([1,8,11,11]))
    for i in range(0,4):
        if i == 3:
            ind = data.sample1.loc[idx][i].index(1)
            t_1[0,ind,:,:]=1
        else:
            for (x,y) in data.sample1.loc[idx][i]:
                t_1[0,i,y+5,x+5]=1
    t_2 = torch.tensor(np.zeros([1,8,11,11]))
    for i in range(0,4):
        if i == 3:
            ind = data.sample2.loc[idx][i].index(1)
            t_2[0,ind,:,:]=1
        else:
            for (x,y) in data.sample2.loc[idx][i]:
                t_2[0,i,y+5,x+5]=1
    t_3 = torch.tensor(np.zeros([1,8,11,11]))
    for i in range(0,4):
        if i == 3:
            ind = data.sample3.loc[idx][i].index(1)
            t_3[0,ind,:,:]=1
        else:
            for (x,y) in data.sample3.loc[idx][i]:
                t_3[0,i,y+5,x+5]=1
    t_4 = torch.tensor(np.zeros([1,8,11,11]))
    for i in range(0,4):
        if i == 3:
            ind = data.sample4.loc[idx][i].index(1)
            t_4[0,ind,:,:]=1
        else:
            for (x,y) in data.sample4.loc[idx][i]:
                t_4[0,i,y+5,x+5]=1
    t_5 = torch.tensor(np.zeros([1,8,11,11]))
    for i in range(0,4):
        if i == 3:
            ind = data.sample5.loc[idx][i].index(1)
            t_5[0,ind,:,:]=1
        else:
            for (x,y) in data.sample5.loc[idx][i]:
                t_5[0,i,y+5,x+5]=1
    sample = {'state': Variable(t_q), 's1': Variable(t_1), 's2': Variable(t_2), 's3': Variable(t_3), 's4': Variable(t_4), 's5': Variable(t_5), 'target': torch.tensor(data.direction[idx])}
    return sample


def accuracy_preds(ys, ts):
    predictions = torch.max(ys, 1)[1]
    correct_prediction = torch.eq(predictions, ts)
    return predictions, torch.mean(correct_prediction.float())

def accuracy(ys, ts):
    predictions = torch.max(ys, 1)[1]
    correct_prediction = torch.eq(predictions, ts)
    return torch.mean(correct_prediction.float())

# Function to get label
def get_labels(batch):
    return Variable(batch['target'])



def predict_data(data_old, model):
    data = data_old.copy()
    for idx in data.index:
        min_time = min(data.time)
        sample = transform(data,idx)
        output = model(sample)
        label = torch.max(get_labels(sample), 0)[1]
        predictions, acc = accuracy_preds(output['out'],label)
        data.loc[idx, 'accuracy'] = acc.data.numpy()
        data.loc[idx, 'prediction'] = predictions.data.numpy()
        data.at[idx, 'predict_loc'] = pred_to_action(predictions.data.numpy(), data.loc[idx, 'agent_loc'])
        correct = 0
        if data.at[idx, 'agent_loc'] == data.at[idx, 'true_agent_loc']:
            correct = 1
        data.at[idx,'pos_acc'] = correct
    return data


def pred_to_action(pred, loc):
    if pred == 4:
        return loc
    if pred == 0:
        return (loc[0],loc[1]-1)
    if pred == 1:
        return (loc[0],loc[1]+1)
    if pred == 2:
        return (loc[0]+1,loc[1])
    if pred == 3:
        return (loc[0]-1,loc[1])
    
def dir_to_num(direction):
    if type(direction) == N:
        return 0
    if type(direction) == S:
        return 1
    if type(direction) == E:
        return 2
    if type(direction) == W:
        return 3
    if direction == None:
        return 4
        

def new_state(G, pred_loc, obj_dic, agent_dic, true_time):
    w = []
    o = []
    a = []
    pos = pred_loc
    l = [(x,y) for x in list(range(-5,6)) for y in list(range(-5,6))]
    for loc in l:
        coor = (pos[0]+loc[0],pos[1]+loc[1])
        if coor not in G.nodes:
            w.append(loc)
        elif coor in obj_dic:
            o.append(loc)
        elif (coor in agent_dic[true_time-1]) and (loc!=(0,0)):
            a.append(loc)
    direction_vec = [0,0,0,0,0]
    return [w, o, a, direction_vec]



#create data for two-step ahead prediction
def data_from_prediction(data, G, obj_dic):
    agent_dic = dict()
    for i in data.true_time.unique():
        agent_dic[i] = list(data.predict_loc[data.true_time==i])
    data_2 = data.copy()
    min_time = min(data.time)
    for idx in data_2.index:
        if data_2.loc[idx, 'time'] > min_time:
            #move states down one position
            data_2.at[idx,'sample1'] = data.loc[idx-1,'query_state'].copy()
            data_2.at[idx,'sample2'] = data.loc[idx-1,'sample1'].copy()
            data_2.at[idx,'sample3'] = data.loc[idx-1,'sample2'].copy()
            data_2.at[idx,'sample4'] = data.loc[idx-1,'sample3'].copy()
            data_2.at[idx,'sample5'] = data.loc[idx-1,'sample4'].copy()
            #replace query state by predicted state
            data_2.at[idx,'query_state'] = new_state(G, data.loc[idx-1,'predict_loc'], obj_dic, agent_dic, data.loc[idx,'true_time'])
            direction_vec = [0,0,0,0,0]
            direction_vec[int(data.loc[idx-1,'prediction'])] = 1
            data_2.at[idx,'sample1'][-1] = direction_vec
            data_2.at[idx,'agent_loc'] = data.at[idx-1,'predict_loc']
    data_2 = data_2.drop(list(data_2[data_2.time==min_time].index))
    return data_2

# Function to get input
def get_input(batch):
    return {'s1': Variable(batch['sample1']),
            's2':Variable(batch['sample2']),
            's3':Variable(batch['sample3']),
            's4':Variable(batch['sample4']),
            's5':Variable(batch['sample5']),
            'state': Variable(batch['state'])}


def get_numpy(x):
    """ Get numpy array for both cuda and not. """
    return x.data.numpy()


height, width, channels = (11,11,8)

conv_out_channels =  8 # <-- Filters in your convolutional layer
kernel_size = 3       # <-- Kernel size
conv_stride = 1       # <-- Stride
conv_pad    = 1       # <-- Padding
NUM_CLASSES = 5
lstm_out_channels = 16
 
def conv_dim(dim_size):
    return int(dim_size - kernel_size + 2 * conv_pad / conv_stride + 1)

conv1_h = conv_dim(height)//2
conv1_w = conv_dim(width)//2

# Keep track of features to output layer
features_cat_size = int(lstm_out_channels//2 * conv1_h * conv1_w)

        
features_cat_size2 = int(conv_out_channels*2 * conv1_h * conv1_w)

# <-- Number of features concatenated before output layer
 

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        
        self.conv_1 = Conv2d(in_channels=channels,
                             out_channels=conv_out_channels,
                             kernel_size=kernel_size,
                             stride=conv_stride,
                             padding=conv_pad,
                             bias=True)
        
        self.lstm = ConvLSTM(conv_out_channels, lstm_out_channels, (3,3), 1, True, True, False)
        
        self.pool = nn.AvgPool3d(2, 2, 0)
        
        self.l_sample = Linear(in_features=features_cat_size,
                            out_features=11*11,
                            bias=True)
        
        
        self.conv_2 = Conv2d(in_channels=4,
                             out_channels=conv_out_channels*4,
                             kernel_size=kernel_size,
                             stride=conv_stride,
                             padding=conv_pad,
                             bias=True)
        
        self.conv_3 = Conv2d(in_channels=conv_out_channels*4,
                             out_channels=conv_out_channels*4,
                             kernel_size=kernel_size,
                             stride=conv_stride,
                             padding=conv_pad,
                             bias=True)
        
        
        
        self.dropout = Dropout2d(p=0.5)
        
        self.batch1 = BatchNorm2d(conv_out_channels)
        self.batch2 = BatchNorm2d(conv_out_channels*2)

        
        self.fc = Linear(in_features=features_cat_size2,
                            out_features=100,
                            bias=True)
        
        self.l_out = Linear(in_features=100,
                            out_features=NUM_CLASSES,
                            bias=False)
        
    def forward(self, s):
        features = []
        out = {}
        
        ## Convolutional layer ##
        # - Change dimensions to fit the convolutional layer 
        # - Apply Conv2d
        # - Use an activation function
        # - Change dimensions s.t. the features can be used in the final FFNN output layer
        features_sample = relu(self.conv_1(s['s1'].float()))
        features_sample.unsqueeze_(1)
        
        for i in range(2,6):
            features = relu(self.conv_1(s['s'+str(i)].float()))
            features.unsqueeze_(1)
            features_sample = torch.cat((features_sample,features),1)
            
        #print(features_sample.size())
        #pass features sample into lstm
        _, last_states = self.lstm(features_sample)
        #print(last_states[0][0].size())
        
        pooled_lstm = self.pool(last_states[0][0])
        #print(pooled_lstm.size())
        
        embedding = relu(self.l_sample(pooled_lstm.view(-1,features_cat_size)))
        #print(embedding.size())
        
        embedding = embedding.view(-1,11,11)
        #print(embedding.size())
        
        ## Concat with query state
        query = torch.cat((s['state'].float(),embedding.unsqueeze_(1)),1)
        #print(query.size())
        
        q_features = relu(self.conv_2(query))
        #print(q_features.size())
        
        q_features = self.pool(relu(self.conv_3(q_features)))
        #print(q_features.size())
        
        q_features = q_features.view(-1,features_cat_size2)
        #print(q_features.size())
        
        q_features = relu(self.fc(q_features))
        #print(q_features.size())
        
        ## Output layer where all features are in use ##
        
        
        out['out'] = self.l_out(q_features)
        #print(out['out'].size())
        return out
    
    
class hospitalDataset(Dataset):
    
    def __init__(self, data_path):
        data = pd.read_csv(data_path)
        self.target = data.direction.apply(literal_eval)
        self.sample1 = data.sample1.apply(literal_eval)
        self.sample2 = data.sample2.apply(literal_eval)
        self.sample3 = data.sample3.apply(literal_eval)
        self.sample4 = data.sample4.apply(literal_eval)
        self.sample5 = data.sample5.apply(literal_eval)
        self.query_state = data.query_state.apply(literal_eval)
    def __len__(self):
        return len(self.target)
    def __getitem__(self,idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()
        t_q = torch.tensor(np.zeros([3,11,11]))
        for i in range(0,3):
            for (x,y) in self.query_state.iloc[idx][i]:
                t_q[i,y+5,x+5]=1
        t_1 = torch.tensor(np.zeros([8,11,11]))
        for i in range(0,4):
            if i == 3:
                ind = self.sample1.iloc[idx][i].index(1)
                t_1[ind,:,:]=1
            else:
                for (x,y) in self.sample1.iloc[idx][i]:
                    t_1[i,y+5,x+5]=1
        t_2 = torch.tensor(np.zeros([8,11,11]))
        for i in range(0,4):
            if i == 3:
                ind = self.sample2.iloc[idx][i].index(1)
                t_2[ind,:,:]=1
            else:
                for (x,y) in self.sample2.iloc[idx][i]:
                    t_2[i,y+5,x+5]=1
        t_3 = torch.tensor(np.zeros([8,11,11]))
        for i in range(0,4):
            if i == 3:
                ind = self.sample3.iloc[idx][i].index(1)
                t_3[ind,:,:]=1
            else:
                for (x,y) in self.sample3.iloc[idx][i]:
                    t_3[i,y+5,x+5]=1
        t_4 = torch.tensor(np.zeros([8,11,11]))
        for i in range(0,4):
            if i == 3:
                ind = self.sample4.iloc[idx][i].index(1)
                t_4[ind,:,:]=1
            else:
                for (x,y) in self.sample4.iloc[idx][i]:
                    t_4[i,y+5,x+5]=1
        t_5 = torch.tensor(np.zeros([8,11,11]))
        for i in range(0,4):
            if i == 3:
                ind = self.sample5.iloc[idx][i].index(1)
                t_5[ind,:,:]=1
            else:
                for (x,y) in self.sample5.iloc[idx][i]:
                    t_5[i,y+5,x+5]=1
        sample = {'state': t_q, 'sample1': t_1, 'sample2': t_2, 'sample3': t_3, 'sample4': t_4, 'sample5': t_5, 'target': torch.tensor(self.target[idx])}
        return sample