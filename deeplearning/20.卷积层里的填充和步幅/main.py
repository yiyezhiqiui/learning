import torch
from torch import nn


def comp_conv2d(conv2d, X):
    X = X.reshape((1, 1) + X.shape)
    Y = conv2d(X)
    return Y.reshape(Y.shape[2:])


conv2d = nn.Conv2d(1, 1, kernel_size=3, padding=1)
X = torch.rand(size=(8, 8))
# print(comp_conv2d(conv2d,X).shape)

conv2d = nn.Conv2d(1, 1, kernel_size=(5, 3), padding=(2, 1))
# print(comp_conv2d(conv2d,X).shape)

conv2d = nn.Conv2d(1, 1, kernel_size=3, padding=1, stride=2)
# print(comp_conv2d(conv2d,X).shape)

conv2d = nn.Conv2d(1, 1, kernel_size=3, padding=(0, 1), stride=(3, 4))# 在上下各添加0行，左右各添加1列，在行方向上每次移动3个像素单位，在列方向上4个
# print(comp_conv2d(conv2d,X).shape)
