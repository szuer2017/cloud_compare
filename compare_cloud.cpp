#include  "Compare.h"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointT;
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3  Vector;

typedef std::pair<Point, Vector> PointVectorPair;

#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


int main(int argc, char* argv[]){

	pcl::PointCloud<PointT>::Ptr src(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr sor_pc(new pcl::PointCloud<PointT>);

	//pcl::io::loadPLYFile("F:\Projects\compare_cloud\compare_cloud\bunnyNoise.ply", *src);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>("bunnyNoise.ply", *src) == -1) //* load the file 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		system("PAUSE");
		return (-1);
	}

	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(src);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*sor_pc);

	//compute the normal
	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(sor_pc);

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(sor_pc);
	ne.setSearchMethod(tree);
	ne.setKSearch(25);

	ne.compute(*pcNormal);

	//connection the points and normal
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normal(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*sor_pc, *pcNormal, *cloud_with_normal);

	//save  File  format as  XYZ
	int size_t = cloud_with_normal->size();

	ofstream file("bunny.xyz");
	file.precision(6);
	for (int i = 0; i < size_t; i++)
	{
		std::string error_p = "1.#QNAN0";
		std::string x = to_string(cloud_with_normal->points[i].x);
		std::string y = to_string(cloud_with_normal->points[i].y);
		std::string z = to_string(cloud_with_normal->points[i].z);
		std::string n_x = to_string(cloud_with_normal->points[i].normal_x);
		std::string n_y = to_string(cloud_with_normal->points[i].normal_y);
		std::string n_z = to_string(cloud_with_normal->points[i].normal_z);
		if (x == error_p || n_x == error_p || n_y == error_p || n_y == error_p)
		{
			x = y = z = n_x = n_y = n_z = "0.000000";
		}
		file << x << " "
			<< y << " "
			<< z << " "
			<< n_x << " " << n_y << " " << n_z << endl;
	}

	file.close();

	const char* input_filename = (argc > 1) ? argv[1] : "bunny.xyz";
	const char* output_filename = (argc > 2) ? argv[2] : "bunny_bf.ply";

	std::vector<PointVectorPair> points;
	std::ifstream stream(input_filename);
	if (!stream ||
		!CGAL::read_xyz_points(stream,
		std::back_inserter(points),
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
		normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
	{
		std::cerr << "Error: cannot read file " << input_filename << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "read the all data!" << std::endl;

	// Algorithm parameters
	int k = 100;                 // size of neighborhood. The bigger the smoother the result will be.
	// This value should bigger than 1.
	double sharpness_angle = 20; // control sharpness of the result.
	// The bigger the smoother the result will be
	int iter_number = 3;         // number of times the projection is applied

	for (int i = 0; i < iter_number; ++i)
	{
		/* double error = */
		//std::cout << "bilateral filter START!" << std::endl;
		CGAL::bilateral_smooth_point_set <Concurrency_tag>(
			points,
			k,
			CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
			normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()).
			sharpness_angle(sharpness_angle));
	}

	std::cout << "bilateral filter have done" << endl;

	//save the bilateral filter pointcloud with normal
	std::ofstream ref("bunnybf.ply");
	CGAL::write_ply_points(
		ref, points,
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
		normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));

	return EXIT_SUCCESS;
}