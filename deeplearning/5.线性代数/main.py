import torch

x = torch.tensor([3.0])
y = torch.tensor([2.0])

print(x + y, x * y, x / y, x * y)

# 将向量视为标量组成的列表
x = torch.arange(4)
print(x)
print(x[3])
print(len(x))

A = torch.arange(20).reshape(5, 4)
print(A)
print(A.T)

B = torch.tensor([[1, 2, 3], [2, 0, 4], [3, 4, 5]])
print(B)
print(B == B.T)

X = torch.arange(24).reshape(2, 3, 4)
print(X)

A = torch.arange(20, dtype=torch.float32).reshape(5, 4)
B = A.clone()
print(
    A,
)
print(A + B)

print(A * B)

a = 2
print(a + X)
print((a * X).shape)

A = torch.arange(20 * 2).reshape(2, 5, 4)
print(A)
print(A.shape)
print(A.sum())
A_sum_axis0 = A.sum(axis=0)
print(A_sum_axis0)
print(A_sum_axis0.shape)
A_sum_axis1 = A.sum(axis=1)
print(A_sum_axis1)
print(A_sum_axis1.shape)
print(A.sum(axis=[0, 1]))

A = torch.arange(20, dtype=float).reshape(5, 4)
print(A.mean())
print(A.sum() / A.numel())
print(A.mean(axis=0))
print(A.sum(axis=0) / A.shape[0])

sum_A = A.sum(axis=1, keepdims=True)
print(sum_A)
print(A / sum_A)

print(A.cumsum(axis=0))

x = torch.arange(4, dtype=torch.float32)
y = torch.ones(4, dtype=torch.float32)
print(x)
print(y)
print(torch.dot(x, y))
print(torch.sum(x * y))

A = torch.arange(20, dtype=float).reshape(5, 4)
x = torch.arange(4, dtype=float)
print(A.shape)
print(x.shape)
print(torch.mv(A, x))

B = torch.ones(4, 3, dtype=float)
print(torch.mm(A, B))

u = torch.tensor([3.0, -4.0])
print(torch.norm(u))
print(torch.abs(u).sum())
print(torch.norm(torch.ones(4, 9)))

#---按特定轴求和---------------------------------------

