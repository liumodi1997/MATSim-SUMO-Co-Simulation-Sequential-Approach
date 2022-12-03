import numpy as np
import matplotlib.pyplot as plt

mu = 4500
sigma = 20
gauss = np.random.normal(loc=mu, scale=sigma, size=(20))
print(gauss)