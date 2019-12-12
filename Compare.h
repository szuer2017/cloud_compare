#ifndef COMPARE_H
#define COMPARE_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/convolution_3d.h>
#include <iostream>
#include <CGAL/Simple_cartesian.h>
//#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/tags.h>
#include <utility> // defines std::pair

#include <vtkPLYReader.h>
#include "vtkActor.h"
#include "vtkCellArray.h"
#include "vtkFloatArray.h"
#include "vtkPointData.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include <vtkLookupTable.h>
#include "vtkSmartPointer.h"
#include <vtkParticleReader.h>
#include "vtkContourFilter.h"







#endif