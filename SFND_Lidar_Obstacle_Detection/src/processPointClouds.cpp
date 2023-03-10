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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered ( new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion ( new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
void ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud1, typename pcl::PointCloud<PointT>::Ptr cloud2) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

    for(int idx : inliers->indices)
        planeCloud->points.push_back(cloud1->points[idx]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud1);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    //std::cout << (cloud1->size());
    *cloud1 = *planeCloud;
    //std::cout << "\t" << (cloud1->size())<<std::endl;
    *cloud2 += *obstCloud;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> insideResult;
	srand(time(NULL));

    if(cloud->size() < 3) return insideResult;

    size_t cloudSize =  cloud->size();

    while(maxIterations--)
    {
        std::unordered_set<int> inside;
        for(int i=0; i<3; ++i)
            inside.emplace(rand() % cloudSize);

        if(inside.size()!=3) continue;
        auto it = inside.begin();

        PointT point1 = cloud->points[*it++], point2 = cloud->points[*it++], point3 = cloud->points[*it];

        std::pair<float, std::pair<float, float>> vec1, vec2, normal;
        vec1 = {point2.x - point1.x, {point2.y-point1.y, point2.z-point1.z}};
        vec2 = {point3.x - point1.x, {point3.y-point1.y, point3.z-point1.z}};
        normal = {vec1.second.first * vec2.second.second - vec1.second.second*vec2.second.first,{
                                                vec1.second.second*vec2.first - vec1.first*vec2.second.second,
                                                vec1.first*vec2.second.first - vec1.second.first*vec2.first
                                                }};

        float A, B, C, D;
        A =  normal.first, B = normal.second.first, C = normal.second.second;
        D = -(A * point1.x + B * point1.y + C*point1.z);

        // normal v1Xv2
        for(int nIndex=0; nIndex < cloudSize; ++nIndex)
        {
            if (inside.count(nIndex)>0) continue;
            float d = fabs(A*cloud->points[nIndex].x + B*cloud->points[nIndex].y + C*cloud->points[nIndex].z + D) / sqrt(A*A+B*B+C*C);
            if(d <= distanceTol)
            {
                inside.emplace(nIndex);
            }
        }
        if(insideResult.size() < inside.size())
        {
            insideResult = inside;
        }
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
    	
	return insideResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceThreshold);

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = {cloudInliers, cloudOutliers};

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::pclSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

    int i=0, num_points = (int) cloud->size(), now_points = 0, prev_points = (int) cloud->size();
    while(prev_points-now_points > num_points*0.01)
    {
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        prev_points = (int) cloud->size();
        SeparateClouds(inliers, cloud, obstCloud);
        now_points = (int) cloud->size();
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = {cloud, obstCloud};

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}



template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT>& tree, float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(idx);

    std::vector<int> nearest = tree.search(cloud, idx, distanceTol);

    for(int id : nearest)
    {
        if(processed[id]) continue;
        
        Proximity(id, cloud, cluster, processed, tree, distanceTol);
    }
}



template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>& tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(cloud->points.size(), false);

    for(int i=0; i<cloud->points.size(); ++i)
    {
        if(processed[i]) continue;
        std::vector<int> cluster;

        Proximity(i, cloud, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
    }

	return clusters;
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    KdTree<PointT> tree {};
    tree.init();
    tree.insert_cloud(cloud);

  	std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

    sort(cloudClusters.begin(), cloudClusters.end(), cloud_size());

    typename pcl::PointCloud<PointT>::Ptr prevCluster (new pcl::PointCloud<PointT>);
  	for(std::vector<int> cluster : clusters){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        if (cluster.size() < 2) continue;
        for(int idx : cluster)
            cloudCluster->points.push_back(cloud->points[idx]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        PointT minPoint, maxPoint;
        pcl::getMinMax3D(*cloudCluster, minPoint, maxPoint);

        if((maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y) * (maxPoint.z - minPoint.z) > maxSize)
        {
            std::vector<std::vector<int>> smallClusters = euclideanCluster(cloud, tree, distanceTol*0.8);

            for(std::vector<int> cluster : smallClusters){
                typename pcl::PointCloud<PointT>::Ptr smallCloudCluster (new pcl::PointCloud<PointT>);
                if (cluster.size() < 2) continue;
                for(int idx : cluster)
                    smallCloudCluster->points.push_back(cloud->points[idx]);
                smallCloudCluster->width = smallCloudCluster->points.size();
                smallCloudCluster->height = 1;
                smallCloudCluster->is_dense = true;
                cloudClusters.push_back(smallCloudCluster);
            }

        }
        else if((maxPoint.x - minPoint.x) * (maxPoint.y - minPoint.y) * (maxPoint.z - minPoint.z) < minSize){
            prevCluster = cloudCluster;
        }
        else{
            cloudClusters.push_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return cloudClusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::pclClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> EC;
    EC.setClusterTolerance(clusterTolerance);
    EC.setMinClusterSize(minSize);
    EC.setMaxClusterSize(maxSize);
    EC.setSearchMethod(tree);
    EC.setInputCloud(cloud);
    EC.extract(clusterIndices);

    for(auto getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for(int idx : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[idx]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster){

    // Need To Study
    // I copy this function through internet.(Next time i will fix it)
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorPCA = eigen_solver.eigenvectors();
    eigenVectorPCA.col(2) = eigenVectorPCA.col(0).cross(eigenVectorPCA.col(1));

    typename pcl::PointCloud<PointT>::Ptr cloudPCAprojection (new pcl::PointCloud<PointT>);
    typename pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVectorPCA); 
    const Eigen::Vector3f bboxTransform = eigenVectorPCA * meanDiagonal + pcaCentroid.head<3>();

    BoxQ box;
    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

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
