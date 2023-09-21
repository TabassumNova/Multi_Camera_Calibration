import pandas as pd
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

class Net(nn.Module):
    def __init__(self):
      super(Net, self).__init__()

      # First fully connected layer
      self.fc1 = nn.Linear(6, 1)

    def forward(self, x):

        # Pass data through ``fc1``
        x = self.fc1(x)
        # output = x
        # x = F.relu(x)
        # x = self.dropout2(x)
        # x = self.fc2(x)

        # Apply softmax to x
        # output = F.softmax(x, dim=1)

        m = nn.Sigmoid()
        # input = torch.randn(2)
        # output = m(input)
        x1 = x.reshape(-1)
        output = m(x1)
        return output

def main1():
    xls = pd.ExcelFile(r"D:\MY_DRIVE_N\Masters_thesis\final report\Report_topics\Blender Images\final_img\analyze_all\analysis2.xlsx") # use r before absolute file path

    sheetX = xls.parse(0)

    data = sheetX.values
    X = []
    y = []
    for n in data:
        X.append(n[:-1])
        y.append(n[-1])
    std_list = []
    mean_list = []
    error_list = []
    for row in range(data.shape[0]):
        if data[row][6]<10:
            std_list.append(data[row][3])
            mean_list.append(data[row][5])
            error_list.append(data[row][6])

    plt.scatter(std_list, error_list)
    plt.show()
    return X, y

def main2(X,y):
    model = Net()
    loss_fn = nn.L1Loss()  # binary cross entropy
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    n_epochs = 100
    batch_size = 10

    for epoch in range(n_epochs):
        Xbatch = X
        y_pred = model(Xbatch)
        ybatch = y
        loss = loss_fn(y_pred, ybatch)
        # optimizer.zero_grad(set_to_none=True)
        loss.backward()
        optimizer.step()
        print(f'Finished epoch {epoch}, latest loss {loss}')

    with torch.no_grad():
        y_pred = model(X)

    accuracy = (y_pred.round() == y).float().mean()
    print(f"Accuracy {accuracy}")
    pass




if __name__ == '__main__':
    # x0, y0 = main1()
    # X = torch.tensor(x0, dtype=torch.float32)
    # # y = torch.tensor(y0, dtype=torch.float32).reshape(-1, 1)
    # y = torch.tensor(y0, dtype=torch.float32)
    # main2(X, y)


    pass