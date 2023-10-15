#!/usr/bin/env python3

#import pandas as pd
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
from cust_model import CustomModel

class NN_state_estimator:
    def __init__(self):

        model_path = '/home/aniket/Documents/model_plain_cr_0.pt'
        self.device = torch.device("cuda")
        model = CustomModel()
        model = torch.load(model_path)
        self.model = model
        self.Im_data = torch.zeros((11, 2, 400, 200), dtype=torch.float)
        self.pos_data = torch.zeros((11, 4), dtype=torch.float)
 
    @torch.no_grad()    
    def estimate(self, Image, Pose):
        
        self.Im_data = torch.tensor(Image, dtype=torch.float)
        self.pos_data = torch.tensor(Pose, dtype=torch.float)
        with torch.cuda.amp.autocast():
            pred = self.model(self.Im_data.to(self.device), self.pos_data.to(self.device)).cpu().squeeze()

        self.Im_data.detach()
        self.pos_data.detach()
        torch.cuda.empty_cache()
        return pred
    
    def model_summary(self):
        self.model.summary()

    
    
        

    

