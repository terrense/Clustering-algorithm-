#include <iostream>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZRGB PointT;

// Compute the Euclidean distance between two points
float calculateDistance(const PointT& point1, const PointT& point2) {
    return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2) + std::pow(point1.z - point2.z, 2));
}

// Find all neighbors within the specified radius using KD-Tree
void searchNeighbors(pcl::PointCloud<PointT>::Ptr cloud, pcl::KdTreeFLANN<PointT>& kdtree, PointT& queryPoint, float radius, std::vector<int>& neighbors) {
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(queryPoint, radius, neighbors, pointRadiusSquaredDistance);
}

// DBSCAN clustering algorithm implementation
void performDBSCAN(pcl::PointCloud<PointT>::Ptr cloud, float eps, int minPts, std::vector<int>& clusterLabels) {
    int currentClusterId = 0;
    clusterLabels.assign(cloud->points.size(), -1); // Initialize all labels to -1 (unvisited)
    pcl::KdTreeFLANN<PointT> kdtree; // KD-Tree for fast neighborhood search
    kdtree.setInputCloud(cloud); // Provide the point cloud to the KD-Tree

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (clusterLabels[i] != -1) continue; // Skip already visited points

        std::vector<int> neighbors;
        searchNeighbors(cloud, kdtree, cloud->points[i], eps, neighbors);

        if (neighbors.size() < minPts) {
            clusterLabels[i] = 0; // Mark as noise
        } else {
            currentClusterId++;
            clusterLabels[i] = currentClusterId;

            std::vector<int> seeds = neighbors;
            size_t index = 0;
            while (index < seeds.size()) {
                int currentPoint = seeds[index];

                if (clusterLabels[currentPoint] == 0) {
                    clusterLabels[currentPoint] = currentClusterId; // Change noise to border point
                }
                
                if (clusterLabels[currentPoint] == -1) {
                    clusterLabels[currentPoint] = currentClusterId;
                    std::vector<int> currentNeighbors;
                    searchNeighbors(cloud, kdtree, cloud->points[currentPoint], eps, currentNeighbors);

                    if (currentNeighbors.size() >= minPts) {
                        seeds.insert(seeds.end(), currentNeighbors.begin(), currentNeighbors.end());
                    }
                }
                index++;
            }
        }
    }
}

int main() {
    // Create a point cloud object and populate it with data
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    // Assume the point cloud has been filled with data here

    float epsilon = 0.5; // Radius for neighborhood search
    int minPoints = 10; // Minimum number of points to form a cluster
    std::vector<int> clusterLabels;

    performDBSCAN(cloud, epsilon, minPoints, clusterLabels);

    for (size_t i = 0; i < clusterLabels.size(); ++i) {
        std::cout << "Point " << i << " is assigned to cluster " << clusterLabels[i] << std::endl;
    }

    return 0;
}
