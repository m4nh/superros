from scipy.cluster.hierarchy import dendrogram, linkage,fcluster,centroid,fclusterdata
from scipy.spatial.distance import pdist
import numpy as np


class ClusterBuilder2D(object):

    def __init__(self):
        pass

    @staticmethod
    def clusterPoints(points,distance_th):
        labels =fclusterdata(points,distance_th,criterion='distance')
        n_labels = max(labels)
        labels_counter = np.zeros((n_labels))
        labels_map = {}
        for i in range(0,len(labels)):
            label = labels[i]
            labels_counter[label-1]+=1
            if label not in labels_map:
                labels_map[label] = np.array(points[i,:])
            else:
                labels_map[label] = np.vstack((labels_map[label],points[i,:]))
        return labels_map
