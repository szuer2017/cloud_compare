#include "Compare.h"
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <iostream>
#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
typedef PointCloud<PointXYZ> Cloud;
typedef PointXYZ PointType;

void compute(Cloud &cloud_a, Cloud &cloud_b)
{
	// Estimate
	TicToc tt;
	tt.tic();

	print_highlight(stderr, "Computing ");

	std::vector<float>  dist_AB;
	// compare A to B
	pcl::search::KdTree<PointType> tree_b;
	tree_b.setInputCloud(cloud_b.makeShared());
	float max_dist_a = -std::numeric_limits<float>::max();
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);

		tree_b.nearestKSearch(cloud_a.points[i], 1, indices, sqr_distances);
		if (sqr_distances[0] > max_dist_a)
			max_dist_a = sqr_distances[0];


		dist_AB.push_back(max_dist_a);

	}

	pcl::PointCloud<pcl::PointXYZI> cloud_i;
	cloud_i.width = cloud_a.size();
	cloud_i.height = 1;
	cloud_i.points.resize(cloud_i.width  *   cloud_i.height);
	for (size_t i = 0; i < cloud_a.points.size(); ++i)
	{
		cloud_i.points[i].x = cloud_a.points[i].x;
		cloud_i.points[i].y = cloud_a.points[i].y;
		cloud_i.points[i].z = cloud_a.points[i].z;
		cloud_i.points[i].intensity = std::sqrt(dist_AB[i]);
	}
	pcl::io::savePLYFileASCII("cloud_new.ply", cloud_i);

	// compare B to A
	pcl::search::KdTree<PointType> tree_a;
	tree_a.setInputCloud(cloud_a.makeShared());
	float max_dist_b = -std::numeric_limits<float>::max();
	for (size_t i = 0; i < cloud_b.points.size(); ++i)
	{
		std::vector<int> indices(1);
		std::vector<float> sqr_distances(1);

		tree_a.nearestKSearch(cloud_b.points[i], 1, indices, sqr_distances);


		if (sqr_distances[0] > max_dist_b)
			max_dist_b = sqr_distances[0];
	}

	max_dist_a = std::sqrt(max_dist_a);
	max_dist_b = std::sqrt(max_dist_b);

	float dist = std::max(max_dist_a, max_dist_b);

	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : ");
	print_info("A->B: "); print_value("%f", max_dist_a);
	print_info(", B->A: "); print_value("%f", max_dist_b);
	print_info(", Hausdorff Distance: "); print_value("%f", dist);
	print_info(" ]\n");

}

int main(int argc, char** argv){

	print_info("Compute Hausdorff distance between point clouds. For more information, use: %s -h\n", argv[0]);


	Cloud::Ptr cloud_a(new Cloud);
	pcl::io::loadPLYFile("bunny1.ply", *cloud_a);

	Cloud::Ptr cloud_b(new Cloud);
	pcl::io::loadPLYFile("bunnybf1.ply", *cloud_b);

	//compute(*cloud_a, *cloud_b);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPLYFile("cloud_new.ply", *cloud_i);

	int num = cloud_i->points.size();
	std::vector<float> temp;
	
	for (int i = 0; i < num; i++)
	{
		//temp.push_back(cloud_i->points[i].intensity);
// 		cloud_out->points[i].x = cloud_i->points[i].x;
// 		cloud_out->points[i].y = cloud_i->points[i].y;
// 		cloud_out->points[i].z = cloud_i->points[i].z;
		pcl::PointXYZRGB temp_p;
		temp_p.x = cloud_i->points[i].x;
		temp_p.y = cloud_i->points[i].y;
		temp_p.z = cloud_i->points[i].z;
		if (cloud_i->points[i].intensity<0.2)
		{
			temp_p.r = 0;
			temp_p.g = 255;
			temp_p.b = 63;
		}
		else
		{
			temp_p.r = 255;
			temp_p.g = 127;
			temp_p.b = 0;
		}
		cloud_out->push_back(temp_p);
	}
	pcl::io::savePCDFile("cloud_new1.pcd", *cloud_out);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(src, 0.6,0.2, 0.5);
	viewer->setBackgroundColor(0, 0, 0); //ÉèÖÃ±³¾°
	viewer->addPointCloud<pcl::PointXYZRGB >(cloud_out, "src");



	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}



	system("pause");

}
