import torch
import torch.nn as nn
import torch.optim as optim

class CustomModel(nn.Module):
    def __init__(self):
        super(CustomModel, self).__init__()    
        # Adjust the size here based on new input size
        self.fc1 = nn.Sequential(
            nn.Linear(2*100 * 40, 64), 
            nn.ReLU(),
        )
        
        
        self.fc2 = nn.Sequential(
            nn.Linear(64, 32),
            nn.ReLU(),
        )
        
        self.fc3 = nn.Sequential(
            nn.Linear(32, 8),
            nn.ReLU(),
        )
        
        # The additional input size is 4, no need to change this.
        self.additional_fc = nn.Sequential(
            nn.Linear(4, 4),
            nn.ReLU(),
        )
        
        self.concat_fc = nn.Sequential(
            nn.Linear(8 + 4, 8),
            nn.ReLU(),
        )
        
        # No activation function as this is a regression task
        self.output_fc = nn.Linear(8, 2)
        
    def forward(self, inputs, additional_input):
        # Flatten the inputs tensor from (2,100,40) to (8000,)
        inputs = inputs.view(inputs.size(0), -1)
        
        x = self.fc1(inputs)
        x = self.fc2(x)
        x = self.fc3(x)
        
        #y = self.additional_fc(additional_input)
        
        #concatenated = torch.cat((x, y), dim=1)
        #x = self.concat_fc(concatenated)
        
        # This will contain the predictions for pitch and roll
        output = self.output_fc(x)
        
        return output

#model = CustomModel()
