import torch

# 数据操作-----------------------------------
x = torch.arange(20)
print(x)
x.shape
print(x.shape)
print(x.numel())
x = x.reshape(4, 5)
print(x)
print(torch.zeros((2, 3, 4)))
print(torch.ones((2, 3, 4)))

x = torch.tensor([1.0, 2, 4, 8])
y = torch.tensor(
    [
        2,
        2,
        2,
        2,
    ]
)
print(x + y)
print(x - y)
print(x * y)
print(x / y)
print(x**y)

x = torch.arange(12, dtype=torch.float32).reshape(3, 4)
y = torch.tensor([[2.0, 1, 4, 3], [1, 2, 3, 4], [4, 3, 2, 1]])
print(torch.cat((x, y), dim=0))
print(torch.cat((x, y), dim=1))
print(x == y)
print(x.sum())

a = torch.arange(3).reshape((3, 1))
b = torch.arange(4).reshape((1, 4))
print(a)
print(b)
print(a + b)

print(x[-1])
print(x[1:3])

print(x[1, 2])
x[0:2, :] = 12
print(x)

X = 5
Y = 10
before = id(Y)
Y = Y + X
print(id(Y) == before)

A = x.numpy()
B = torch.tensor(A)
print(type(A))
print(type(B))

# 数据预处理--------------------------

import os

os.makedirs(os.path.join("./", "data"), exist_ok=True)
data_file = os.path.join("./", "data/", "house_tine.csv")
with open(data_file, "w") as f:
    f.write("NumRooms,Alley,Price\n")  # 列名
    f.write("NaN,Pave,127500\n")  # 每行表示一个数据样本
    f.write("2,NaN,10600\n")
    f.write("4,NaN,178100\n")
    f.write("NaN,NaN,140000\n")

import pandas as pd

data = pd.read_csv(data_file)
print(data)

inputs, outputs = data.iloc[:, 0:2], data.iloc[:, 2]
inputs = inputs.fillna(inputs.mean(numeric_only=True))
print(inputs)
inputs = pd.get_dummies(inputs, dummy_na=True, dtype=int)
print(inputs)

import torch

x, y = torch.tensor(inputs.values), torch.tensor(outputs.values)
print(x, y)
