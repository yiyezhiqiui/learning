import torch
from torch import nn

# print(torch.device('cpu'),torch.cuda.device('cuda'))
# print(torch.cuda.device_count())


def try_gpu(i=0):
    if torch.cuda.device_count() >= i + 1:
        return torch.device(f"cuda:{i}")
    return torch.device("cpu")


def try__all_gpus():
    devices = [torch.device(f"cuda:{i}") for i in range(torch.cuda.device_count())]
    return devices if devices else [torch.device("cpu")]


# print(try_gpu())
# print(try_gpu(10))
# print(try__all_gpus())

x = torch.tensor([1, 2, 3])
# print(x)

x = torch.ones(2,3,device=try_gpu())
# print(x)

net = nn.Sequential(nn.Linear(3,1))
net = net.to(device=try_gpu())
# print(net(x))

print(net[0].weight.data.device)