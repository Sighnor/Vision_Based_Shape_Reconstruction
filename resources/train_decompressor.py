import sys
import struct

import numpy as np

import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.tensorboard import SummaryWriter

from train_common import load_positions, load_rotations, save_network

# Networks

class Decompressor(nn.Module):

    def __init__(self, input_size, output_size, hidden_size=512):
        super(Decompressor, self).__init__()
        
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
    
    Pos = load_positions('./positions.bin')['positions'].copy().astype(np.float32)
    Rot = load_rotations('./rotations.bin')['rotations'].copy().astype(np.float32)
    
    size = Pos.shape[0]
    npositions = Pos.shape[1]
    nrotations = Rot.shape[1]
    
    # Parameters
    
    seed = 1234
    batchsize = 32
    lr = 0.001
    niter = 500000
    
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.set_num_threads(1)
    
    # Compute means/stds
    
    Pos_scale = Pos.std()
    Pos_noise_std = Pos.std(axis=0) + 1.0
    
    decompressor_mean_out = torch.as_tensor(np.hstack([
        Rot.mean(axis=0).ravel(),
    ]).astype(np.float32))
    
    decompressor_std_out = torch.as_tensor(np.hstack([
        Rot.std(axis=0).ravel(),
    ]).astype(np.float32))
    
    decompressor_mean_in = torch.as_tensor(np.hstack([
        Pos.mean(axis=0).ravel(),
    ]).astype(np.float32))
    
    decompressor_std_in = torch.as_tensor(np.hstack([
        Pos_scale.repeat(npositions),
    ]).astype(np.float32))
    
    # Make networks
    
    network_decompressor = Decompressor(npositions, nrotations)
    
    # Function to generate test predictions

    def generate_predictions():
        
        with torch.no_grad():
            
            # Get slice of database for first clip
            
            start = 0
            stop = size
            
            Poshat = Pos[start:stop]
            
            # Find nearest
            
            Rotgnd = torch.as_tensor(Rot[Poshat])
            
            # Decompress
            
            output = (network_decompressor((Poshat - decompressor_mean_in) / decompressor_std_in) *
                decompressor_std_out + decompressor_mean_out)
            
            Rottil = output[:,:]
            
            # Write features
            
            fmin, fmax = Poshat.cpu().numpy().min(), Poshat.cpu().numpy().max()
            
            fig, axs = plt.subplots(npositions, sharex=True, figsize=(12, 2*npositions))
            for i in range(npositions):
                axs[i].plot(Poshat[:500:4,i].cpu().numpy(), marker='.', linestyle='None')
                axs[i].set_ylim(fmin, fmax)
            plt.tight_layout()
            
            try:
                plt.savefig('decompressor_Pos.png')
            except IOError as e:
                print(e)

            plt.close()
            
            # Write rotations
            
            lmin, lmax = Rotgnd.cpu().numpy().min(), Rotgnd.cpu().numpy().max()
            
            fig, axs = plt.subplots(nrotations, sharex=True, figsize=(12, 2*nrotations))
            for i in range(nrotations):
                axs[i].plot(Rotgnd[:500:4,i].cpu().numpy(), marker='.', linestyle='None')
                axs[i].plot(Rottil[:500:4,i].cpu().numpy(), marker='.', linestyle='None')
                axs[i].set_ylim(lmin, lmax)
            plt.tight_layout()
            
            try:
                plt.savefig('decompressor_Rot.png')
            except IOError as e:
                print(e)

            plt.close()

    # Train
    
    writer = SummaryWriter()

    optimizer = torch.optim.AdamW(
        network_decompressor.parameters(), 
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
        
        Poshat = Pos[samples]
        
        # Find rot
        
        Rotgnd = torch.as_tensor(Rot[samples])
        
        # Decompressor
        
        output = (network_decompressor((Poshat - decompressor_mean_in) / decompressor_std_in) *
            decompressor_std_out + decompressor_mean_out)
        
        Rottil = output[:,:]
        
        # Compute Losses
        
        loss_rotval = torch.mean(5.0 * torch.abs(Rotgnd - Rottil))
        loss = loss_rotval
        
        # Backprop
        
        loss.backward()

        optimizer.step()
    
        # Logging
        
        writer.add_scalar('decompressor/loss', loss.item(), i)
        
        writer.add_scalars('decompressor/loss_terms', {
            'rotval': loss_rotval.item(),
        }, i)
        
        if rolling_loss is None:
            rolling_loss = loss.item()
        else:
            rolling_loss = rolling_loss * 0.99 + loss.item() * 0.01
        
        if i % 10 == 0:
            sys.stdout.write('\rIter: %7i Loss: %5.3f' % (i, rolling_loss))
        
        if i % 1000 == 0:
            generate_predictions()
            save_network('decompressor.bin', [
                network_decompressor.linear0, 
                network_decompressor.linear1, 
                network_decompressor.linear2, 
                network_decompressor.linear3,
                network_decompressor.linear4],
                decompressor_mean_in,
                decompressor_std_in,
                decompressor_mean_out,
                decompressor_std_out)
            
        if i % 1000 == 0:
            scheduler.step()
            