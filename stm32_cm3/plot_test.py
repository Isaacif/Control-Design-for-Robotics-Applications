import pandas as pd
import matplotlib.pyplot as plt

dados = pd.read_csv("data_Vol.csv")
time = []
t = 0
for elem in dados.values.tolist():
    time.append(t)
    t+=0.1

plt.plot(time, dados.values.tolist())
plt.show()