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

void compute(Cloud &cloud_a, Cloud &cloud_b )
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
		cloud_i.points[i].x = cloud_a.points[i].x  ;
		cloud_i.points[i].y = cloud_a.points[i].y ;
		cloud_i.points[i].z = cloud_a.points[i].z;
		cloud_i.points[i].intensity = std::sqrt(dist_AB[i]) ;
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

int main(int argc ,char** argv){

	print_info("Compute Hausdorff distance between point clouds. For more information, use: %s -h\n", argv[0]);


	Cloud::Ptr cloud_a(new Cloud);
	pcl::io::loadPLYFile("bunny1.ply", *cloud_a);

	Cloud::Ptr cloud_b(new Cloud);
	pcl::io::loadPLYFile("bunnybf1.ply", *cloud_b);

	compute (*cloud_a, *cloud_b);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPLYFile("cloud_new.ply", *cloud_i);

#if 0
	vtkSmartPointer<vtkParticleReader> cube = vtkSmartPointer<vtkParticleReader>::New();
	cube->SetFileName("F:\\Projects\\compare_cloud\\compare_cloud\\cloud_new.ply");
	cube->SetDataByteOrderToBigEndian();
	cube->Update();
#endif

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < cloud_i->size(); i++)
	{ 
		points->InsertPoint(i, cloud_i->points[i].x, cloud_i->points[i].y, cloud_i->points[i].z);
	}

	vtkSmartPointer<vtkPolyData> cube =  vtkSmartPointer<vtkPolyData>::New();
	cube->SetPoints(points);

	vtkSmartPointer<vtkContourFilter> contour = vtkSmartPointer<vtkContourFilter>::New();
	contour->SetInputData(cube);
	contour->Update();
	vtkSmartPointer<vtkPolyData>points1 = contour->GetOutput();

    vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();

	std::vector<float> temp;
	for (int i = 0; i < cloud_i->points.size(); i++)
	{
		temp.push_back(cloud_i->points[i].intensity);
	
		if (temp[i]<0.1335 || temp[i]>0.20256)
		{
			scalars->InsertTuple1(i, 0);
		}
		else
		{
			scalars->InsertTuple1(i, floor(1000*(temp[i] - 0.133) / 15));
		}
	}
	
		cube->GetPointData()->SetScalars(scalars);

#if 0
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	std::string  filename = "buuny1.ply";
	reader->SetFileName(filename.c_str());
	reader->Update();
#endif

  //定义颜色映射表
	vtkSmartPointer<vtkLookupTable> pColorTable = vtkSmartPointer<vtkLookupTable>::New();
	pColorTable->SetNumberOfColors(15);
	pColorTable->SetTableValue(0, 1.0, 0.0, 0.0, 1.0);
	pColorTable->SetTableValue(1, 1.0, 0.34902, 0.0, 1.0);
	pColorTable->SetTableValue(2, 1.0, 0.57647, 0.0, 1.0);
	pColorTable->SetTableValue(3, 1.0, 0.8000, 0.0, 1.0);
	pColorTable->SetTableValue(4, 1.0, 0.92548, 0.0, 1.0);
	pColorTable->SetTableValue(5, 0.82645, 1.0, 0.0, 1.0);
	pColorTable->SetTableValue(6, 0.45282, 1.0, 0.0, 1.0);
	pColorTable->SetTableValue(7, 0.09804, 1.0, 0.0, 1.0);
	pColorTable->SetTableValue(8, 0.0, 1.0, 0.62536, 1.0);
	pColorTable->SetTableValue(9, 0.0, 1.0, 0.87451, 1.0);
	pColorTable->SetTableValue(10, 0.0, 0.67235, 1.0, 1.0);
	pColorTable->SetTableValue(11, 0.0, 0.57902, 1.0, 1.0);
	pColorTable->SetTableValue(12, 0.0, 0.40000, 1.0, 1.0);
	pColorTable->SetTableValue(13, 0.0, 0.28804, 1.0, 1.0);
	pColorTable->SetTableValue(14, 0.0, 0.00000, 1.0, 1.0);
	//pColorTable->SetHueRange(0.67, 0.0);        //色调范围从红色到蓝色
	pColorTable->Build();

	//数据映射
	vtkSmartPointer<vtkPolyDataMapper>  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputConnection(cube->GetOutputPort());
	mapper->SetInputData(cube);
	mapper->SetScalarRange(0,69);//采用误差距离值的最大，最小值作为区间
	mapper->SetLookupTable(pColorTable);
	
	vtkSmartPointer<vtkActor>  cubeActor = vtkSmartPointer<vtkActor>::New();
	cubeActor->SetMapper(mapper);

	vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(render);
	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);
	render->AddActor(cubeActor);

	render->SetBackground(0.1,0.1,0.1);
	renWin->SetSize(720, 640);
	renWin->Render();
	iren->Start();



	system("pause");
	 
}
