import sys
import struct

import numpy as np

import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.tensorboard import SummaryWriter

from train_common import load_rotations, load_positions, save_network

# Networks

class Compressor(nn.Module):

    def __init__(self, input_size, output_size, hidden_size=512):
        super(Compressor, self).__init__()
        
        self.linear0 = nn.Linear(input_size, hidden_size)
        self.linear1 = nn.Linear(hidden_size, hidden_size)
        self.linear2 = nn.Linear(hidden_size, hidden_size)
        self.linear3 = nn.Linear(hidden_size, hidden_size)
        self.linear4 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        x = F.relu(self.linear0(x))
        x = F.relu(self.linear1(x))
        x = F.relu(self.linear2(x))
        x = F.relu(self.linear3(x))
        return self.linear4(x)

# Training procedure

if __name__ == '__main__':
    
    # Load data
    
    Rot = load_rotations('./rotations.bin')['rotations'].copy().astype(np.float32)
    Pos = load_positions('./positions.bin')['positions'].copy().astype(np.float32)
    
    size = Rot.shape[0]
    nrotations = Rot.shape[1]
    npositions = Pos.shape[1]
    
    # Parameters
    
    seed = 1234
    batchsize = 32
    lr = 0.001
    niter = 500000
    
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(1)
    
    # Compute means/stds
    
    Rot_scale = Rot.std()
    Rot_noise_std = Rot.std(axis=0) + 1.0
    
    compressor_mean_out = torch.as_tensor(np.hstack([
        Pos.mean(axis=0).ravel(),
    ]).astype(np.float32))
    
    compressor_std_out = torch.as_tensor(np.hstack([
        Pos.std(axis=0).ravel(),
    ]).astype(np.float32))
    
    compressor_mean_in = torch.as_tensor(np.hstack([
        Rot.mean(axis=0).ravel(),
    ]).astype(np.float32))
    
    compressor_std_in = torch.as_tensor(np.hstack([
        Rot_scale.repeat(nrotations),
    ]).astype(np.float32))
    
    # Make networks
    
    network_compressor = Compressor(nrotations, npositions)
    
    # Function to generate test predictions

    def generate_predictions():
        
        with torch.no_grad():
            
            # Get slice of database for first clip
            
            start = 0
            stop = size
            
            Rothat = Rot[start:stop]
            
            # Find nearest
            
            Posgnd = torch.as_tensor(Pos[Rothat])
            
            # Decompress
            
            output = (network_compressor((Rothat - compressor_mean_in) / compressor_std_in) *
                compressor_std_out + compressor_mean_out)
            
            Postil = output[:,:]
            
            # Write features
            
            fmin, fmax = Rothat.cpu().numpy().min(), Rothat.cpu().numpy().max()
            
            fig, axs = plt.subplots(nrotations, sharex=True, figsize=(12, 2*nrotations))
            for i in range(nrotations):
                axs[i].plot(Rothat[:500:4,i].cpu().numpy(), marker='.', linestyle='None')
                axs[i].set_ylim(fmin, fmax)
            plt.tight_layout()
            
            try:
                plt.savefig('compressor_Rot.png')
            except IOError as e:
                print(e)

            plt.close()
            
            # Write positions
            
            lmin, lmax = Posgnd.cpu().numpy().min(), Posgnd.cpu().numpy().max()
            
            fig, axs = plt.subplots(npositions, sharex=True, figsize=(12, 2*npositions))
            for i in range(npositions):
                axs[i].plot(Posgnd[:500:4,i].cpu().numpy(), marker='.', linestyle='None')
                axs[i].plot(Postil[:500:4,i].cpu().numpy(), marker='.', linestyle='None')
                axs[i].set_ylim(lmin, lmax)
            plt.tight_layout()
            
            try:
                plt.savefig('compressor_Pos.png')
            except IOError as e:
                print(e)

            plt.close()

    # Train
    
    writer = SummaryWriter()

    optimizer = torch.optim.AdamW(
        network_compressor.parameters(), 
        lr=lr,
        amsgrad=True,
        weight_decay=0.001)
        
    scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.99)
    
    rolling_loss = None
    
    sys.stdout.write('\n')
    
    for i in range(niter):
    
        optimizer.zero_grad()
        
        # Extract batch
        
        samples = np.random.randint(0, size, size=[batchsize])
        
        Rothat = Rot[samples]
        
        # Find pos
        
        Posgnd = torch.as_tensor(Pos[samples])
        
        # Compressor
        
        output = (network_compressor((Rothat - compressor_mean_in) / compressor_std_in) *
            compressor_std_out + compressor_mean_out)
        
        Postil = output[:,:]
        
        # Compute Losses
        
        loss_posval = torch.mean(5.0 * torch.abs(Posgnd - Postil))
        loss = loss_posval
        
        # Backprop
        
        loss.backward()

        optimizer.step()
    
        # Logging
        
        writer.add_scalar('compressor/loss', loss.item(), i)
        
        writer.add_scalars('compressor/loss_terms', {
            'posval': loss_posval.item(),
        }, i)
        
        if rolling_loss is None:
            rolling_loss = loss.item()
        else:
            rolling_loss = rolling_loss * 0.99 + loss.item() * 0.01
        
        if i % 10 == 0:
            sys.stdout.write('\rIter: %7i Loss: %5.3f' % (i, rolling_loss))
        
        if i % 1000 == 0:
            generate_predictions()
            save_network('compressor.bin', [
                network_compressor.linear0, 
                network_compressor.linear1, 
                network_compressor.linear2, 
                network_compressor.linear3,
                network_compressor.linear4],
                compressor_mean_in,
                compressor_std_in,
                compressor_mean_out,
                compressor_std_out)
            
        if i % 1000 == 0:
            scheduler.step()
            