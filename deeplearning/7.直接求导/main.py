import torch

x = torch.arange(4.0)
print(x)
x.requires_grad_(True)
y = 2 * torch.dot(x, x)
print(y)

# 通过调用反向传播函数来自动计算y关于x每个分量的梯度
y.backward()
print(x.grad)

# 在默认情况下,Pytorch会累积梯度，我们需要清除之前的值
x.grad.zero_()
y = x.sum()
y.backward()
print('y.sum:',y)
print('x.grad:',x.grad)

# 对非标量调用‘backward’需要传入一个gradient参数，该参数指定微分函数
x.grad.zero_()
y = x * x
# 等价于y.backward(torch.ones(len(x)))
y.sum().backward()
print(x.grad)

x.grad.zero_()
y = x * x
u = y.detach()
z = u * x
z.sum().backward()
print(x.grad == u)

x.grad.zero_()
y.sum().backward()
print(x.grad == 2 * x)


def f(a):
    b = a * 2
    while b.norm() < 1000:
        b = b * 2
    if b.sum() > 0:
        c = b
    else:
        c = 100 * b
    return c


a = torch.randn(size=(), requires_grad=True)
d = f(a)
d.backward()
print(a.grad == d / a)