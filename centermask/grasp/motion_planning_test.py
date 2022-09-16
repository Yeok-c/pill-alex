import numpy as np


def return_pushing_list(centroids, destination):
    centroids = [(700, 396), (711, 339), (716, 298), (775, 305), (592, 268)]
    destination = (705, 470)

    def vector_length_and_angle(p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = np.sqrt(dx ** 2 + dy ** 2)
        theta = np.arctan2(dy, dx)
        return length, theta

    def extend_vector(point, angle, offset):
        extension = (offset*np.cos(angle), offset*np.sin(angle))
        return (point[0]+extension[0], point[1]+extension[1])

    centroid_info = []
    offset = 20
    for centroid in centroids:
        length, angle = vector_length_and_angle(centroid, destination)
        starting_pos = extend_vector(centroid, angle, offset)    
        centroid_info.append((length, starting_pos))

    centroid_info = sorted(centroid_info, key=lambda x: x[0], reverse=True)
    return centroid_info