#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <limits>

typedef pcl::PointXYZRGB PointT;

// Calculate the Euclidean distance between two points
float calculateDistance(const PointT& point1, const PointT& point2) {
    return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
}

// K-Means clustering algorithm implementation
void performKMeansClustering(pcl::PointCloud<PointT>::Ptr cloud, int k, std::vector<int>& clusterLabels) {
    std::vector<PointT> clusterCenters(k);
    std::vector<int> pointsInCluster(k, 0);
    clusterLabels.resize(cloud->points.size(), -1);

    // Randomly initialize the cluster centers
    for (int i = 0; i < k; ++i) {
        clusterCenters[i] = cloud->points[rand() % cloud->points.size()];
    }

    bool isConverged = false;
    while (!isConverged) {
        isConverged = true;

        // Assign points to the nearest cluster center
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            float minDist = std::numeric_limits<float>::max();
            int nearestCluster = -1;

            for (int j = 0; j < k; ++j) {
                float dist = calculateDistance(cloud->points[i], clusterCenters[j]);
                if (dist < minDist) {
                    minDist = dist;
                    nearestCluster = j;
                }
            }

            if (clusterLabels[i] != nearestCluster) {
                isConverged = false;
                clusterLabels[i] = nearestCluster;
            }
        }

        // Recalculate the cluster centers
        clusterCenters.assign(k, PointT());
        pointsInCluster.assign(k, 0);

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            clusterCenters[clusterLabels[i]].x += cloud->points[i].x;
            clusterCenters[clusterLabels[i]].y += cloud->points[i].y;
            clusterCenters[clusterLabels[i]].z += cloud->points[i].z;
            clusterCenters[clusterLabels[i]].r += cloud->points[i].r;
            clusterCenters[clusterLabels[i]].g += cloud->points[i].g;
            clusterCenters[clusterLabels[i]].b += cloud->points[i].b;
            pointsInCluster[clusterLabels[i]]++;
        }

        for (int j = 0; j < k; ++j) {
            if (pointsInCluster[j] != 0) {
                clusterCenters[j].x /= pointsInCluster[j];
                clusterCenters[j].y /= pointsInCluster[j];
                clusterCenters[j].z /= pointsInCluster[j];
                clusterCenters[j].r /= pointsInCluster[j];
                clusterCenters[j].g /= pointsInCluster[j];
                clusterCenters[j].b /= pointsInCluster[j];
            }
        }
    }
}

int main() {
    // Create a point cloud object and populate it with data
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    // Assume the point cloud has been filled with data here

    int numberOfClusters = 5; // Number of clusters to form
    std::vector<int> clusterLabels;

    performKMeansClustering(cloud, numberOfClusters, clusterLabels);

    for (size_t i = 0; i < clusterLabels.size(); ++i) {
        std::cout << "Point " << i << " is assigned to cluster " << clusterLabels[i] << std::endl;
    }

    return 0;
}
