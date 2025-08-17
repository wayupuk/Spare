import torch.nn as nn
import torch.nn.functional as f
from torch.utils.data import Dataset,DataLoader
from torch.optim.lr_scheduler import StepLR,ReduceLROnPlateau
from scipy import signal
import os
from collections import Counter
import random
from sklearn.preprocessing import StandardScaler, LabelEncoder
import torch
import numpy as np
import tqdm
class CNNTimeSeriesClassifier(nn.Module):
    def __init__(self, input_shape, n_classes, dropout=0.3):
        """
        CNN-based Time Series Classifier following your architecture diagram
        
        Args:
            input_shape: Tuple (sequence_length, n_features) - e.g., (121, 21)
            n_classes: Number of output classes
            dropout: Dropout rate
        """
        super(CNNTimeSeriesClassifier, self).__init__()
        
        seq_len, n_features = input_shape
        self.input_shape = input_shape
        self.n_classes = n_classes
        self.dropout_rate = dropout
        
        # Input normalization layer
        self.normalization = nn.BatchNorm1d(n_features)
        
        # Reshape for 2D convolution: (batch, channels, height, width)
        # We'll treat sequence as height and features as width, with 1 channel
        # Input shape: (None, 1, seq_len, n_features) - e.g., (None, 1, 121, 21)
        # First Conv2D block
        self.conv1 = nn.Conv2d(
            in_channels=1, 
            out_channels=32, 
            kernel_size=(3, 3), 
            padding=1
        )
        self.pool1 = nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
        
        # Calculate shape after first conv+pool
        # After conv1: (None, 32, 121, 21) -> (None, 32, 119, 19) with padding=1
        # After pool1: (None, 32, 119, 19) -> (None, 32, 59, 9)
        h1, w1 = seq_len // 2, n_features // 2
        
        # Second Conv2D block
        self.conv2 = nn.Conv2d(
            in_channels=32, 
            out_channels=64, 
            kernel_size=(3, 3), 
            padding=1
        )
        self.pool2 = nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
        
        # Calculate shape after second conv+pool
        # After conv2: (None, 64, 59, 9) -> (None, 64, 57, 7) with padding=1
        # After pool2: (None, 64, 57, 7) -> (None, 64, 28, 3)
        h2, w2 = h1 // 2, w1 // 2
        
        # Third Conv2D block
        self.conv3 = nn.Conv2d(
            in_channels=64, 
            out_channels=64, 
            kernel_size=(3, 3), 
            padding=1
        )
        
        # Calculate final conv output shape
        # After conv3: (None, 64, 28, 3) -> (None, 64, 26, 1) with padding=1
        h3, w3 = h2, w2
        
        # Calculate flattened size dynamically
        self.flatten_size = 64 * h3 * w3
        
        # Fully connected layers
        self.fc1 = nn.Linear(self.flatten_size, 64)
        self.fc2 = nn.Linear(64, n_classes)
        
        # Activation and regularization
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(dropout)
        
        # Initialize weights
        self._init_weights()
    
    def _init_weights(self):
        """Initialize model weights properly"""
        for module in self.modules():
            if isinstance(module, nn.Conv2d):
                nn.init.kaiming_normal_(module.weight, mode='fan_out', nonlinearity='relu')
                if module.bias is not None:
                    nn.init.zeros_(module.bias)
            elif isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                if module.bias is not None:
                    nn.init.zeros_(module.bias)
            elif isinstance(module, nn.BatchNorm1d):
                nn.init.ones_(module.weight)
                nn.init.zeros_(module.bias)
    
    def forward(self, x):
        # Input shape: (batch_size, seq_len, n_features)
        batch_size = x.size(0)
        
        # Normalize along feature dimension
        # Reshape for BatchNorm1d: (batch_size * seq_len, n_features)
        x_norm = x.view(-1, x.size(2))
        # print(x_norm)
        # ([print(i.dtype) for i in x_norm])
        x_norm = self.normalization(x_norm)
        x = x_norm.view(batch_size, x.size(1), x.size(2))
        
        # Reshape for 2D convolution: (batch_size, 1, seq_len, n_features)
        x = x.unsqueeze(1)
        
        # First Conv2D + MaxPool2D
        x = self.conv1(x)
        x = self.relu(x)
        x = self.pool1(x)
        
        # Second Conv2D + MaxPool2D
        x = self.conv2(x)
        x = self.relu(x)
        x = self.pool2(x)
        
        # Third Conv2D
        x = self.conv3(x)
        x = self.relu(x)
        
        # Flatten
        x = x.view(batch_size, -1)
        
        # First Dense layer
        x = self.fc1(x)
        x = self.relu(x)
        x = self.dropout(x)
        
        # Second Dense layer (output)
        x = self.fc2(x)
        
        return x
    
    def get_config(self):
        """Return model configuration for saving"""
        return {
            'input_shape': self.input_shape,
            'n_classes': self.n_classes,
            'dropout': self.dropout_rate,
            'flatten_size': self.flatten_size
        }
        
        
        
class ImprovedCustomDataset(Dataset):
    def __init__(self, dataframe, chunk_size=121, label_encoder=None, is_test=False, 
                 balance_classes=False, min_sequence_length=50,zero_ratio=1,down_zero=True):
        """
        Improved dataset class for CNN time series classification
        
        Args:
            dataframe: Input dataframe
            chunk_size: Sequence length for time series (121 to match your diagram)
            label_encoder: Pre-fitted label encoder (for test data)
            is_test: Whether this is test data
            balance_classes: Whether to balance classes
            min_sequence_length: Minimum sequence length to keep
        """
        self.chunk_size = chunk_size
        self.min_sequence_length = min_sequence_length
        
        # Clean data
        self.data = dataframe.copy()
        self.data = self.data[~self.data.Label.isin(["cooldown", "error_redo", "break_time"])]
        
        # Handle label encoding
        if label_encoder is None:
            self.label_encoder = LabelEncoder()
            self.data["Label"] = self.label_encoder.fit_transform(self.data["Label"])
        else:
            self.label_encoder = label_encoder
            # Handle unseen labels in test data
            known_labels = set(label_encoder.classes_)
            mask = self.data["Label"].isin(known_labels)
            if not mask.all():
                print(f"Warning: Removing {(~mask).sum()} samples with unknown labels")
                self.data = self.data[mask]
            self.data["Label"] = label_encoder.transform(self.data["Label"])
        
        self.n_classes = len(self.label_encoder.classes_)
        
        # Remove timestamp if present
        if "timestamp_ms" in self.data.columns:
            self.data = self.data.drop(columns=["timestamp_ms"])
        
        # Convert to sequences
        self.sequences, self.labels = self._create_sequences()
        
        # Balance classes if requested and not test data
        if balance_classes and not is_test:
            self.sequences, self.labels = self._balance_classes()
        
        if down_zero and not is_test:
            self.sequences, self.labels = self._down_zero(zero_ratio = zero_ratio)
            
        
        print(f"Dataset created: {len(self.sequences)} sequences of shape {self.sequences.shape}")
        print(f"Classes: {self.n_classes}, Distribution: {Counter(self.labels.numpy())}")
    
    def _create_sequences(self):
        """Create sequences from grouped data"""
        # Group by consecutive labels
        self.data['group_id'] = (self.data['Label'] != self.data['Label'].shift()).cumsum()
        grouped_data = [group.drop('group_id', axis=1) for _, group in self.data.groupby('group_id')]
        
        print(f"Found {len(grouped_data)} label groups")
        
        # Filter by minimum length
        valid_groups = [group for group in grouped_data if len(group) >= self.min_sequence_length]
        print(f"Kept {len(valid_groups)} groups after length filtering")
        
        sequences = []
        labels = []
        
        for group in tqdm(valid_groups, desc="Processing sequences"):
            # Separate features and labels
            features = group.drop('Label', axis=1).values
            group_labels = group['Label'].values
            
            # Get the most common label in the sequence
            most_common_label = Counter(group_labels).most_common(1)[0][0]
            
            # Create fixed-length sequence
            if len(features) >= self.chunk_size:
                # If longer than chunk_size, use uniform sampling
                indices = np.linspace(0, len(features)-1, self.chunk_size, dtype=int)
                sequence = features[indices]
            else:
                # If shorter, pad with zeros at the end
                sequence = np.zeros((self.chunk_size, features.shape[1]))
                sequence[:len(features)] = features
            
            sequences.append(sequence)
            labels.append(most_common_label)
        
        return torch.FloatTensor(sequences), torch.LongTensor(labels)
    
    def _balance_classes(self):
        """Balance classes by undersampling majority classes"""
        # Get class counts
        unique_labels, counts = torch.unique(self.labels, return_counts=True)

        min_count = counts.min().item()
        
        print(f"Balancing classes to {min_count} samples each")
        
        balanced_sequences = []
        balanced_labels = []
        
        for label,counts in zip(unique_labels,counts):
            # Get indices for this class
            class_indices = (self.labels == label).nonzero(as_tuple=True)[0]
            # Sample min_count indices
            # if len(class_indices) > min_count and label.item() == 0:
            #     sampled_indices = class_indices[torch.randperm(len(class_indices))[:int(counts.item() * zero_ratio )]]
            if len(class_indices) > min_count:
                sampled_indices = class_indices[torch.randperm(len(class_indices))[:min_count]]
            else:
                sampled_indices = class_indices
            
            balanced_sequences.append(self.sequences[sampled_indices])
            balanced_labels.append(self.labels[sampled_indices])
        
        return torch.cat(balanced_sequences), torch.cat(balanced_labels)
    
    def _down_zero(self,zero_ratio = 1):
        """Balance classes by undersampling majority classes"""
        # Get class counts
        unique_labels, counts = torch.unique(self.labels, return_counts=True)

        
        print(f"down zero with {zero_ratio} ratio")
        
        balanced_sequences = []
        balanced_labels = []
        
        for label,counts in zip(unique_labels,counts):
            # Get indices for this class
            class_indices = (self.labels == label).nonzero(as_tuple=True)[0]
            # Sample min_count indices
            if label.item() == 0:
                sampled_indices = class_indices[torch.randperm(len(class_indices))[:int(counts.item() * zero_ratio )]]
            else:
                sampled_indices = class_indices
            
            balanced_sequences.append(self.sequences[sampled_indices])
            balanced_labels.append(self.labels[sampled_indices])
        
        return torch.cat(balanced_sequences), torch.cat(balanced_labels)
    
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        return self.sequences[idx], self.labels[idx]
    
    def get_info(self):
        """Return dataset information"""
        return {
            'n_samples': len(self.sequences),
            'sequence_length': self.chunk_size,
            'n_features': self.sequences.shape[2],
            'n_classes': self.n_classes,
            'label_encoder': self.label_encoder,
            'class_names': self.label_encoder.classes_,
            'input_shape': (self.chunk_size, self.sequences.shape[2])
        }
        
def convert_data(features):
    chunk_size = 50
    # features = torch.rand(30,28)
    # sequences = []
    print(chunk_size,features.shape)
    
    if len(features) >= 50:
        # If longer than chunk_size, use uniform sampling
        indices = np.linspace(0, len(features)-1, chunk_size, dtype=int)
        sequence = features[indices]
    else:
        # If shorter, pad with zeros at the end
        sequence = np.zeros((chunk_size, features.shape[1]))
        sequence[:len(features)] = features

    
    if torch.cuda.is_available():
        return torch.tensor(sequence).to("cuda")
    else:
        return torch.tensor(sequence)