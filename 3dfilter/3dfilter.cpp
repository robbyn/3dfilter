#include "stdafx.h"
#pragma warning(disable:4996)
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/filter_indices.h>
#include "GroupFilter.h"

typedef pcl::PointXYZRGBNormal PointT;

pcl::PointCloud<PointT>::Ptr
load(const char *fileName)
{
	pcl::PLYReader reader;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	reader.read(fileName, *cloud);
	return cloud;
}

pcl::PointCloud<PointT>::Ptr
cylinderCrop(pcl::PointCloud<PointT>::Ptr cloud, double yc, double zc, double R)
{
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	double R2 = R*R;
	for (size_t i = 0; i < cloud->size(); ++i) {
		PointT &pt = (*cloud)[i];
		double dy = pt.y - yc;
		double dz = pt.z - zc;
		double r2 = dy*dy + dz*dz;
		if (r2 < R2) {
			cloud2->push_back(pt);
		}
	}
	return cloud2;
}

pcl::PointCloud<PointT>::Ptr
radiusOutliersRemoval(pcl::PointCloud<PointT>::Ptr cloud, double r, int neighbours)
{
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::RadiusOutlierRemoval<PointT> rorfilter;
	rorfilter.setInputCloud(cloud);
	rorfilter.setRadiusSearch(r);
	rorfilter.setMinNeighborsInRadius(neighbours);
	rorfilter.filter(*cloud2);
	return cloud2;
}

pcl::PointCloud<PointT>::Ptr
statsOutliersRemoval(pcl::PointCloud<PointT>::Ptr cloud, int meank, double threshold)
{
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::StatisticalOutlierRemoval<PointT> filter;
	filter.setInputCloud(cloud);
	filter.setMeanK(meank);
	filter.setStddevMulThresh(threshold);
	filter.filter(*cloud2);
	return cloud2;
}

pcl::PointCloud<PointT>::Ptr
groupsOutliersRemoval(pcl::PointCloud<PointT>::Ptr cloud, double minDist, int minGroupSize)
{
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::GroupFilter<PointT> filter(minDist,minGroupSize);
	filter.setInputCloud(cloud);
	filter.setRadius(minDist);
	filter.setMinSize(minGroupSize);
	filter.filter(*cloud2);
	return cloud2;
}

bool
readChan(const char *fileName, double &yc, double &zc, double &R)
{
	std::ifstream ifs;
	ifs.open(fileName);
	std::vector<double> ys;
	std::vector<double> zs;
	double ysum = 0;
	double zsum = 0;
	double count = 0;
	while (!ifs.eof())
	{
		char rest[1024];
		int n;
		double x, y, z;
		ifs >> n;
		ifs >> x;
		ifs >> y;
		ifs >> z;
		ifs.getline(rest, sizeof rest/sizeof(char));
		if (ifs.eof())
		{
			break;
		}
		ys.push_back(y);
		zs.push_back(z);
		++count;
		ysum += y;
		zsum += z;
	}

	std::cerr << "Number of cameras: " << count << std::endl;
	if (count == 0)
	{
		return false;
	}
	yc = ysum / count;
	zc = zsum / count;
	double r2sum = 0;
	for (int i = 0; i < count; ++i)
	{
		double dy = ys[i] - yc;
		double dz = zs[i] - zc;
		r2sum += dy*dy + dz*dz;
	}
	R = sqrt(r2sum/count);
	return true;
}

int
secs(clock_t t)
{
	return (int)((double)t / CLOCKS_PER_SEC);
}

int
main(int argc, char** argv)
{
	const char *fileName = NULL;
	const char *outCloud = NULL;
	const char *outMesh = "mesh.ply";

	bool cyl = false;
	double yc = 0.39577119586776877;
	double zc = 5.765876350413221;
	double R = 2.5;
	double cylRatio = 0.8;

	bool outliers = false;
	double r = 0.1;
	int neighbours = 5;

	bool stat = false;
	int meank = 100;
	double threshold = 1;

	bool groups = false;
	double groupDist = 0.1;
	int groupThreshold = 200;

	bool poisson = false;
	int poissonDepth;

	int st = 0;
	for (int i = 1; i < argc; ++i)
	{
		const char *arg = *++argv;
		switch (st) {
		case 0:
			if (strcmp(arg, "--outliers-radius") == 0)
			{
				st = 1;
			}
			else if (strcmp(arg, "--outliers-neighbours") == 0)
			{ 
				st = 2;
			}
			else if (strcmp(arg, "--cyl-yc") == 0)
			{
				st = 3;
			}
			else if (strcmp(arg, "--cyl-zc") == 0)
			{
				st = 4;
			}
			else if (strcmp(arg, "--cyl-radius") == 0)
			{
				st = 5;
			}
			else if (strcmp(arg, "--stat-meank") == 0)
			{
				st = 6;
			}
			else if (strcmp(arg, "--stat-threshold") == 0)
			{
				st = 7;
			}
			else if (strcmp(arg, "--cameras") == 0)
			{
				st = 8;
			}
			else if (strcmp(arg, "--output-cloud") == 0)
			{
				st = 9;
			}
			else if (strcmp(arg, "--cyl-radius-ratio") == 0)
			{
				st = 10;
			}
			else if (strcmp(arg, "--poisson-depth") == 0)
			{
				st = 11;
			}
			else if (strcmp(arg, "--output-mesh") == 0)
			{
				st = 12;
			}
			else if (strcmp(arg, "--groups-radius") == 0)
			{
				st = 13;
			}
			else if (strcmp(arg, "--groups-threshold") == 0)
			{
				st = 14;
			}
			else
			{
				fileName = arg;
			}
			break;
		case 1:
			r = atof(arg);
			st = 0;
			outliers = true;
			break;
		case 2:
			neighbours = atoi(arg);
			st = 0;
			outliers = true;
			break;
		case 3:
			yc = atof(arg);
			st = 0;
			break;
		case 4:
			zc = atof(arg);
			st = 0;
			break;
		case 5:
			R = atof(arg);
			st = 0;
			break;
		case 6:
			meank = atoi(arg);
			st = 0;
			stat = true;
			break;
		case 7:
			threshold = atof(arg);
			st = 0;
			stat = true;
			break;
		case 8:
			cyl = readChan(arg, yc, zc, R);
			st = 0;
			break;
		case 9:
			outCloud = arg;
			st = 0;
			break;
		case 10:
			cylRatio = atof(arg);
			if (cylRatio > 1) {
				cylRatio /= 100.0;
			}
			st = 0;
			break;
		case 11:
			poissonDepth = atoi(arg);
			poisson = true;
			st = 0;
			break;
		case 12:
			outMesh = arg;
			st = 0;
			break;
		case 13:
			groupDist = atof(arg);
			groups = true;
			st = 0;
			break;
		case 14:
			groupThreshold = atoi(arg);
			groups = true;
			st = 0;
			break;
		}
	}
	if (!fileName)
	{
		std::cerr << "No file to process" << std::endl;
		return 0;
	}
	std::cerr << "Load file: " << fileName << std::endl;
	clock_t t = clock();
	pcl::PointCloud<PointT>::Ptr cloud = load(fileName);
	t = clock() - t;
	std::cerr << "PointCloud has: " << cloud->size() << " data points." << " (" << secs(t) << ")" <<  std::endl;
	if (cyl)
	{
		std::cerr << "Cylinder crop, yc: " << yc << ", zc: " << zc << ", R: " << (R*cylRatio) << std::endl;
		t = clock();
		cloud = cylinderCrop(cloud, yc, zc, R*cylRatio);
		t = clock() - t;
		std::cerr << "PointCloud has: " << cloud->size() << " data points." << " (" << secs(t) << ")" << std::endl;
	}
	if (outliers)
	{
		std::cerr << "Radius outliers removal, r: " << r << ", neighbours: " << neighbours << std::endl;
		t = clock();
		cloud = radiusOutliersRemoval(cloud, r, neighbours);
		t = clock() - t;
		std::cerr << "PointCloud has: " << cloud->size() << " data points." << " (" << secs(t) << ")" << std::endl;
	}
	if (stat)
	{
		std::cerr << "Stats outliers removal, meanK: " << meank << ", theshold: " << threshold << std::endl;
		t = clock();
		cloud = statsOutliersRemoval(cloud, meank, threshold);
		t = clock() - t;
		std::cerr << "PointCloud has: " << cloud->size() << " data points." << " (" << secs(t) << ")" << std::endl;
	}
	if (groups)
	{
		std::cerr << "Groups outliers removal, min distance: " << groupDist << ", theshold: " << groupThreshold << std::endl;
		t = clock();
		cloud = groupsOutliersRemoval(cloud, groupDist, groupThreshold);
		t = clock() - t;
		std::cerr << "PointCloud has: " << cloud->size() << " data points." << " (" << secs(t) << ")" << std::endl;
	}
	if (outCloud)
	{
		std::cerr << "Write PointCloud to: " << outCloud << std::endl;
		t = clock();
		pcl::PLYWriter writer;
		writer.write(outCloud, *cloud, false, true);
		t = clock() - t;
		std::cerr << "PointCloud written to " << outCloud << " (" << secs(t) << ")" << std::endl;
	}
	if (poisson)
	{
		std::cerr << "Poisson surface reconstruction, depth: " << poissonDepth << std::endl;
		t = clock();
		pcl::Poisson<PointT> poisson;
		poisson.setDepth(poissonDepth);
		poisson.setInputCloud(cloud);
		pcl::PolygonMesh mesh;
		poisson.performReconstruction(mesh);
		t = clock() - t;
		std::cerr << "Mesh has: " << mesh.polygons.size() << " polygons." << " (" << secs(t) << ")" << std::endl;

		std::cerr << "Write mesh to: " << outMesh << std::endl;
		t = clock();
		pcl::io::savePLYFileBinary(outMesh, mesh);
		t = clock() - t;
		std::cerr << "Mesh written to " << outMesh << " (" << secs(t) << ")" << std::endl;
	}
	return 0;
}
