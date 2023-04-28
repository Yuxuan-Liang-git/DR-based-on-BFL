import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(0, 10, 40)
y = x ** 2 * np.exp(-x)
u = np.array([x[i + 1] - x[i] for i in range(len(x) - 1)])
v = np.array([y[i + 1] - y[i] for i in range(len(x) - 1)])
x = x[:len(u)]  # 使得维数和u,v一致
y = y[:len(v)]
c = np.random.randn(len(u))  # arrow颜色
plt.figure()
plt.quiver(x, y, u, v, c, angles='xy', scale_units='xy', scale=1)
plt.show()
