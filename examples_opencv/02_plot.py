# pip install matplotlib
#
# https://www.datacamp.com/community/tutorials/matplotlib-tutorial-python
# https://matplotlib.org/stable/tutorials/index.html

from matplotlib import pyplot as plt

import numpy as np

# zeichte ein paar Punkte
plt.plot([1, 5, 7], [0, 2, 1], '*', label='points')

# Werte von 0 bis 10 mit Schrittweite 0.3
x = np.arange(0, 10, 0.3)
y = np.sin(x)

# Plot the data
plt.plot(x, x, label='linear')
plt.plot(x, y, label='sin')

# Add a legend
plt.legend()

# Show the plot
plt.show()

