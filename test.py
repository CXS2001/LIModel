import torch

print("CUDA可用:", torch.cuda.is_available())
print("CUDA版本:", torch.version.cuda)
print("当前设备:", torch.cuda.current_device())