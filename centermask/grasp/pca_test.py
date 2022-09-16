## Given up because even if we find the 'optimal swiping angle'
# It may end up pushing objects to too close the border
# Might as well always swipe towards center

# Initial idea:
# 1. Find 2D mean of centroids, subtract from centroids
# 2. Perform 2D PCA on mean-centered centroids
# 3. Take coordinates of axis of 2nd component of PCA
# 4. Swipe according vectors calculated from:
#       - Vector distance calculated from mean + variance size
#       - Vector angle calculated from PCA axis


import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
import numpy as np

mean = np.empty((0))
centroids = [(700, 396), (711, 339), (716, 298), (775, 305), (592, 268)]
centroids=np.array([list(item) for item in centroids])
mean = np.mean(centroids, axis=0)

# centroids = centroids-mean

pca = PCA(n_components=2)
pca.fit(centroids)
# print(pca.components_)
# print(pca.explained_variance_)


def draw_vector(v0, v1, ax=None):
    ax = ax or plt.gca()
    arrowprops=dict(arrowstyle='->',
                    linewidth=2,
                    shrinkA=0, shrinkB=0)
    ax.annotate('', v1, v0, arrowprops=arrowprops)

# plot data
fig, ax = plt.subplots(1,1)
plt.scatter(centroids[:, 0], centroids[:, 1], alpha=0.2)
for length, vector in zip(pca.explained_variance_, pca.components_):
    v = vector * 3 * np.sqrt(length)
    draw_vector(pca.mean_, pca.mean_ + v, ax)

plt.show()