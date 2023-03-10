/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData2D()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add outliers
  	int numOutliers = 200;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 20*rx;
  		point.y = 20*ry;
  		point.z = 0;

        if (rand()%101 < 70) point.y = (rand()%100) / 100;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	int numpoints = 10000;
  	while(numpoints--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double rz = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 20*rx;
  		point.y = 20*ry;
  		point.z = 20*rz;

        if (rand()%101 < 90) point.z = (rand()%100) / 100;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::pair<float, std::pair<float, float>> crossProduct(std::pair<float, std::pair<float, float>> v1, std::pair<float, std::pair<float, float>> v2)
{
    return {v1.second.first * v2.second.second - v1.second.second*v2.second.first,{
                                                v1.second.second*v2.first - v1.first*v2.second.second,
                                                v1.first*v2.second.first - v1.second.first*v2.second.second
                                                }};
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

    std::cout << cloud->size() << std::endl;

    size_t cloudSize =  cloud->size();

    while(maxIterations--)
    {
        std::unordered_set<int> inliers;
        for(int i=0; i<3; ++i)
            inliers.emplace(rand() % cloudSize);

        if(inliers.size()!=3) continue;

        auto it = inliers.begin();

        pcl::PointXYZ point1 = cloud->points[*it++], point2 = cloud->points[*it++], point3 = cloud->points[*it];

        std::pair<float, std::pair<float, float>> vec1, vec2, normal;
        vec1 = {point2.x - point1.x, {point2.y-point1.y, point2.z-point1.z}};
        vec2 = {point3.x - point1.x, {point3.y-point1.y, point3.z-point1.z}};
        normal = crossProduct(vec1, vec2);

        float A, B, C, D;
        A =  normal.first, B = normal.second.first, C = normal.second.second;
        D = -(A * point1.x + B * point1.y + C*point1.z);

        // normal v1Xv2
        for(int nIndex=0; nIndex < cloudSize; ++nIndex)
        {
            if (inliers.count(nIndex)>0) continue;
            float d = fabs(A*cloud->points[nIndex].x + B*cloud->points[nIndex].y + C*cloud->points[nIndex].z + D) / sqrt(A*A+B*B+C*C);
            if(d <= distanceTol)
            {
                inliers.emplace(nIndex);
            }
        }
        if(inliersResult.size() < inliers.size())
        {
            inliersResult = inliers;
        }
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    cout << "Ransac took " << elapsedTime.count() << " milliseconds" << endl;
    	
	return inliersResult;


}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

	// TODO: Fill in this function
	// For max iterations 

    // cloudSize to get random index
    int cloudSize = cloud->size();

    while(maxIterations--)
    {
        std::unordered_set<int> inliers;

        // randomly sample subset
        // select two points.(Line Ransac)
        for(int i=0; i<2; ++i)
            inliers.emplace(rand() % cloudSize);

        // set cant contain same key
        // if inliers size is not two select same key in random sample
        if(inliers.size()!=2) continue;

        // take out two point that are in the form of pcl::PointXYZ format
        auto it = inliers.begin();
        pcl::PointXYZ point1 = cloud->points[*it++], point2 = cloud->points[*it];


        // make line AX+BY+C = 0
        float A = point1.y - point2.y;
        float B = point2.x - point1.x;
        float C = point1.x*point2.y - point2.x*point1.y;

        // iterate all points
        for(int nIndex=0; nIndex < cloudSize; ++nIndex)
        {
            // skip select subset points
            if (inliers.count(nIndex)>0) continue;

            // calculate distance
            float d = fabs(A*cloud->points[nIndex].x + B*cloud->points[nIndex].y + C) / sqrt(A*A+B*B);

            // if distance < distanceTol
            if(d <= distanceTol)
            {
                // it is inliers
                inliers.emplace(nIndex);
            }
        }

        // want to extract plain(biggest one)
        if(inliersResult.size() < inliers.size())
        {
            inliersResult = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    cout << "Ransac took " << elapsedTime.count() << " milliseconds" << endl;
    	
	return inliersResult;
}



int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    viewer->getRenderWindow()->GlobalWarningDisplayOff();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData2D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac2D(cloud, 100, 0.8);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

    viewer->spin();
  	
}
