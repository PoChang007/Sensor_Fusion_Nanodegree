// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // do voxel grid point reduction and region based filtering
    // create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud <PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    // do crop box
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud <PointT>());
    pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*cloud_region);

    // remove points on the roof
    std::vector<int> indices;
    pcl::CropBox<PointT> rootFilter(true);
    rootFilter.setMin(Eigen::Vector4f (-2.5, -1.7, -1, 1));
    rootFilter.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    rootFilter.setInputCloud(cloud_region);
    rootFilter.filter(indices);

    // segment out outliers
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

    for (int index : indices)
    {
        outliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(outliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateCloudsPCL(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>);

    for (auto index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstacleCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateCloudsPCL(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            clusterCloud->points.push_back(cloud->points[index]);

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        clusters.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Ransac algorithm to segment plane and obstacles
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>());
    std::unordered_set<int> inlier_index;
    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> inlier_index_temp;
        while (inlier_index_temp.size() < 3)
        {
            int random_index = rand() % cloud->points.size();
            if (!(inlier_index_temp.count(random_index) > 0))
            {
                inlier_index_temp.insert(random_index);
            }
        }

        // three points
        auto it1 = inlier_index_temp.begin();
        float x1 = cloud->points[*it1].x;
        float y1 = cloud->points[*it1].y;
        float z1 = cloud->points[*it1].z;
        it1++;
        float x2 = cloud->points[*it1].x;
        float y2 = cloud->points[*it1].y;
        float z2 = cloud->points[*it1].z;
        it1++;
        float x3 = cloud->points[*it1].x;
        float y3 = cloud->points[*it1].y;
        float z3 = cloud->points[*it1].z;

        float x2x1 = x2 - x1;
        float x3x1 = x3 - x1;
        float y2y1 = y2 - y1;
        float y3y1 = y3 - y1;
        float z2z1 = z2 - z1;
        float z3z1 = z3 - z1;

        std::vector<float> cross_product = {(y2y1 * z3z1) - (z2z1 * y3y1),
                                            (z2z1 * x3x1) - (x2x1 * z3z1),
                                            (x2x1 * y3y1) - (y2y1 * x3x1)};

        // Calculate the plane based on the given 3D point
        float a = cross_product[0];
        float b = cross_product[1];
        float c = cross_product[2];
        float d = -(cross_product[0] * x1 + cross_product[1] * y1 +
                    cross_product[2] * z1);

        for (int j = 0; j < cloud->points.size(); j++)
        {
            float numerator = fabs(a * cloud->points[j].x + b * cloud->points[j].y + c * cloud->points[j].z + d);
            float denominator = sqrtf(a * a + b * b + c * c);
            float distance_to_plane = numerator / denominator;
            if (distance_to_plane < distanceTol)
                inlier_index_temp.insert(j);
        }

        if (inlier_index_temp.size() > inlier_index.size())
            inlier_index = inlier_index_temp;
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inlier_index.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster,
                                           std::vector<bool> &isPointProcessed, int index, KdTree* tree, float distanceTol)
{
    // mark point as processed
    isPointProcessed[index] = true;
    cluster.push_back(index);
    std::vector<float> point = {cloud->points[index].x, cloud->points[index].y, cloud->points[index].z};
    std::vector<int> nearby = tree->search(point, distanceTol);
    for (int nearbyIndex : nearby)
    {
        if (isPointProcessed[nearbyIndex] != true)
            proximity(cloud, cluster, isPointProcessed, nearbyIndex, tree, distanceTol);
    }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    KdTree *tree = new KdTree();

    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(point, i);
    }

    // return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::vector<bool> isPointProcessed(cloud->points.size(), false);

    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (isPointProcessed[i] != true)
        {
            // create cluster
            std::vector<int> cluster;
            proximity(cloud, cluster, isPointProcessed, i, tree, distanceTol);
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
            {
                clusters.push_back(cluster);
            }
        }
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersCloud;

    for (std::vector<int> cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice : cluster)
        {
            clusterCloud->points.push_back(cloud->points[indice]);
        }

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        clustersCloud.push_back(clusterCloud);
    }

    return clustersCloud;
}