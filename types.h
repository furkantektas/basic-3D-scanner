/* 
 * File:   types.h
 * Author: furkan
 *
 */

#ifndef TYPES_H
#define	TYPES_H

#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>


// Points
//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBA PointXYZRGBA;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBNormal PointTNormal;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//Clouds
typedef pcl::PointCloud <PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//Color Clouds
typedef pcl::PointCloud <PointXYZRGBA> ColorCloud;
typedef ColorCloud::Ptr ColorCloudPtr;
typedef ColorCloud::ConstPtr ColorCloudConstPtr;

//Gray Clouds
typedef pcl::PointCloud <PointXYZ> GrayCloud;
typedef GrayCloud::Ptr GrayCloudPtr;
typedef GrayCloud::ConstPtr GrayCloudConstPtr;


//Normals
typedef pcl::PointCloud <PointTNormal> CloudNormal;
typedef CloudNormal::Ptr CloudNormalPtr;
typedef CloudNormal::ConstPtr CloudNormalConstPtr;

// Matrixes
typedef Eigen::Matrix <bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
typedef Eigen::MatrixXi MatrixXi;

//Meshes
typedef pcl::PolygonMesh Mesh;

typedef pcl::Vertices Vertices;
#endif	/* TYPES_H */

