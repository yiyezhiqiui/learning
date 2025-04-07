import torch
from torch import nn
from torch.nn import functional as F

x = torch.arange(4)
torch.save(x, "./file/x-file")

x2 = torch.load("./file/x-file")
# print(x2)

y = torch.zeros(4)
torch.save([x, y], "./file/x-file")
x2, y2 = torch.load("./file/x-file")
# print(x2, y2)

mydict = {"x": x, "y": y}
torch.save(mydict, "./file/mydict")
mydict2 = torch.load("./file/mydict")
# print(mydict2)


class MLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.hidden = nn.Linear(20, 256)
        self.output = nn.Linear(256, 10)

    def forward(self, x):
        return self.output(F.relu(self.hidden(x)))


net = MLP()
X = torch.randn(size=(2, 20))
Y = net(X)

torch.save(net.state_dict(), "./file/mlp.params")

clone = MLP()
clone.load_state_dict(torch.load("./file/mlp.params"))
print(clone.eval())

Y_clone = clone(X)
print(Y_clone == Y)
