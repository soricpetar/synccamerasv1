#!/home/s_petar/miniconda3/envs/clean_seminar/bin/python
import torch 
import torch.nn as nn
import torch.nn.functional as F

from mvn.models.triangulation import RANSACTriangulationNet, AlgebraicTriangulationNet, VolumetricTriangulationNet
from mvn.models.loss import KeypointsMSELoss, KeypointsMSESmoothLoss, KeypointsMAELoss, KeypointsL2Loss, VolumetricCELoss

from mvn.utils import cfg
config = cfg.load_config('../myconfig.yaml')
device = torch.device('cpu')

model = VolumetricTriangulationNet(config, device = device)
if config.model.init_weights:
        state_dict = torch.load(config.model.checkpoint, map_location="cpu")
        for key in list(state_dict.keys()):
            new_key = key.replace("module.", "")
            state_dict[new_key] = state_dict.pop(key)

        model.load_state_dict(state_dict, strict=True)
        print("Successfully loaded pretrained weights for whole model")

#print(model)



