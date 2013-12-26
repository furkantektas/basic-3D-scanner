/*
 * File:   PCLOperations.h
 * Author: furkan
 *
 */

#ifndef PCLOPERATIONS_H
#define	PCLOPERATIONS_H

#include "types.h"
#include <pcl/pcl_exports.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <vector>
#include <string>
#include <sstream>


#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/filter.h>


#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>


#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

using std::vector;

class PCLOperations {
    static const double RAD_PER_DEGREE = 0.0174532925;
    public:
    static void clip(ColorCloudPtr& cloud,ColorCloudPtr& cloud_out, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) {

        // Create a cloud if it's null
        if (!cloud_out)
        cloud_out = ColorCloudPtr(new ColorCloud());
      
#ifdef __linux__
        
        pcl::PassThrough<PointXYZRGBA> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (y_min, y_max);
        pass.filter (*cloud_out);
 
        pass.setInputCloud (cloud_out);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x_min, x_max);
        pass.filter (*cloud_out);
 
        pass.setInputCloud (cloud_out);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (z_min, z_max);
        pass.filter (*cloud_out);
        
#else
        

        // cm -> m for the comparison
        const float x_min = .01f * _x_min;
        const float x_max = .01f * _x_max;
        const float y_min = .01f * _y_min;
        const float y_max = .01f * _y_max;
        const float z_min = .01f * _z_min;
        const float z_max = .01f * _z_max;

        cloud_out->reserve(cloud->size());
        
        PointXYZRGBA nan;
        nan.x = nan.y = nan.z = std::numeric_limits <float>::quiet_NaN();
        for(ColorCloud::const_iterator i = cloud->begin(); i != cloud->end(); ++i) {
            if( boost::math::isnan(i->x) ||
                i->x <= x_min || i->x >= x_max ||
                i->y <= y_min || i->y >= y_max ||
                i->z <= z_min || i->z >= z_max)
                cloud_out->push_back(nan);
                //true;
            else
                cloud_out->push_back(*i);
        }
        
        cloud_out->width = cloud->width;
        cloud_out->height = cloud->height;
        cloud_out->is_dense = false;
        
#endif
    }
    
    static void voxelGrid(ColorCloudPtr& cloud) {
        pcl::VoxelGrid<PointXYZRGBA> grid;
        grid.setLeafSize (0.001, 0.001, 0.001);
        grid.setInputCloud (cloud);
        grid.filter (*cloud);
    }
    
    static void radiusOutlierRemoval(ColorCloudPtr& cloud) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.001);
        outrem.setMinNeighborsInRadius (30);
        // apply filter
        outrem.filter (*cloud);
    }
    
    static pcl::PointCloud<PointT>::Ptr polynomialReconstruction(pcl::PointCloud<PointT>::Ptr cloud){

            // Create a KD-Tree
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);

            // Output has the PointNormal type in order to store the normals calculated by MLS
             pcl::PointCloud<pcl::PointNormal> mls_points;

            // Init object (second point type is for the normals, even if unused)
            pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls;

            mls.setComputeNormals (true);

            // Set parameters
             mls.setInputCloud (cloud);
             mls.setPolynomialFit (true);
             mls.setSearchMethod (tree);
             mls.setSearchRadius (0.05);

            // Reconstruct
             mls.process (mls_points);

            pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
            pcl::io::loadPCDFile ("bun0-mls.pcd", *cloud);

            return cloud;
    }
    
    static void convertCloudTypes(ColorCloudPtr c1, GrayCloudPtr c2) {
        pcl::copyPointCloud(*c1,*c2);
    }
   
    static Mesh convertToMesh(GrayCloudPtr& tmp) {
        std::cerr << "tmp w:" << tmp->width << " tmp h: " << tmp->height << " tmp s:"  << tmp->size() << std::endl;

        pcl::PointCloud <PointXYZ>::Ptr tmp2 = pcl::PointCloud <PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::copyPointCloud(*tmp,*tmp2);

        std::cerr << "tmp2 w:" << tmp2->width << " tmp2 h: " << tmp2->height << " tmp2 s:"  << tmp2->size() << std::endl;

        // Normal estimation*
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (tmp2);
        n.setInputCloud (tmp2);
        n.setSearchMethod (tree);
        n.setKSearch (50);
        n.compute (*normals);
        //* normals should not contain the point normals + surface curvatures

        std::cerr << "normals are calculated" << std::endl;


        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*tmp2, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        std::cerr << "concatenateFields" << std::endl;

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

        std::cerr << "KdTree" << std::endl;

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        Mesh triangles;

        std::cerr << "GreedyProjectionTriangulation" << std::endl;
        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (0.05);

        std::cerr << "setSearchRadius" << std::endl;
        // Set typical values for the parameters
        gp3.setMu (2.5);
        gp3.setMaximumNearestNeighbors (100);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(false);
        
        std::cerr << "gp3" << std::endl;
        // Get result
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);

        /*
        std::cerr << "reconstruct" << std::endl;
        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();

        std::cerr << "states" << std::endl;
        pcl::io::saveVTKFile ("mesh.vtk", triangles);
        std::cerr << "saved" << std::endl;
        */
        return triangles;
    }

    static void shiftCloud(GrayCloudPtr cloud_in, const float x_mov, const float y_mov, const float z_mov){
        Eigen::Vector3f offset(x_mov,y_mov,z_mov); 
        Eigen::Quaternionf rotation;
        rotation.setIdentity();
        pcl::transformPointCloud(*cloud_in,*cloud_in,offset,rotation);
    }

    static void translateCloud(ColorCloudPtr cloud_in, const float x_mov, const float y_mov, const float z_mov){
        Eigen::Vector3f offset(x_mov,y_mov,z_mov); 
        Eigen::Quaternionf rotation;
        rotation.setIdentity();
        pcl::transformPointCloud(*cloud_in,*cloud_in,offset,rotation);
    }
    
    static void rotateCloud(ColorCloudPtr cloud_in, int stepSize, int count) {
        Eigen::Vector3f offset(0,0,0);
        Eigen::Quaternionf rotation;
        rotation.setIdentity();
        rotation.y() = sin(((M_PI/stepSize*1.0)*count));
        rotation.w() = cos(((M_PI/stepSize*1.0)*count));
        //rotation.w() = toRadian(7.2*count);
        rotation.x() = rotation.z() = 0.0;
        pcl::transformPointCloud(*cloud_in,*cloud_in,offset,rotation);
    }


    static double toRadian(const double degree){
        return degree * PCLOperations::RAD_PER_DEGREE;
    }

    static Eigen::Matrix4f generateTransformMatrixY(const double degree){
        double rad = toRadian(degree);
        Eigen::Matrix4f transform;

        transform <<  cos(rad), 0, sin(rad), 0,
                      0, 1, 0, 0,
                      (-1 * sin(rad)), 0, cos(rad), 0,
                      0,0,0,1;

        return transform;    
    }


    // Define a new point representation for < x, y, z, curvature >
    class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
    {
      using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
    public:
      MyPointRepresentation ()
      {
        // Define the number of dimensions
        nr_dimensions_ = 4;
      }

      // Override the copyToFloatArray method to define our feature vector
      virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
      {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
      }
    };
    
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Align a pair of PointCloud datasets and return the result
      * \param cloud_src the source PointCloud
      * \param cloud_tgt the target PointCloud
      * \param output the resultant aligned source PointCloud
      * \param final_transform the resultant transform between source and target
      */
    void pairAlign (const ColorCloudPtr cloud_src, const ColorCloudPtr cloud_tgt, ColorCloudPtr output, Eigen::Matrix4f &final_transform, bool downsample = false)
    {
        using namespace pcl;
        GrayCloudPtr cloud_src_gray(new GrayCloud),cloud_tgt_gray(new GrayCloud);
        pcl::copyPointCloud(*cloud_src,*cloud_src_gray);
        pcl::copyPointCloud(*cloud_tgt,*cloud_tgt_gray);
        //
        // Downsample for consistency and speed
        // \note enable this for large datasets
        GrayCloudPtr src (new GrayCloud);
        GrayCloudPtr tgt (new GrayCloud);
        pcl::VoxelGrid<PointXYZ> grid;
        if (downsample)
        {
          grid.setLeafSize (0.05, 0.05, 0.05);
          grid.setInputCloud (cloud_src_gray);
          grid.filter (*src);

          grid.setInputCloud (cloud_tgt_gray);
          grid.filter (*tgt);
        }
        else
        {
          src = cloud_src_gray;
          tgt = cloud_tgt_gray;
        }

        //pcl::transformPointCloud(*cloud_tgt_gray, *cloud_tgt_gray, generateTransformMatrixY(7.2));

        // Compute surface normals and curvature
        PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

        pcl::NormalEstimation<PointXYZ, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        norm_est.setSearchMethod (tree);
        norm_est.setKSearch (100);

        norm_est.setInputCloud (src);
        norm_est.compute (*points_with_normals_src);
        pcl::copyPointCloud (*src, *points_with_normals_src);

        norm_est.setInputCloud (tgt);
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

        //
        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation point_representation;
        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues (alpha);

        //
        // Align
        pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
        reg.setTransformationEpsilon (1e-2);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance (0.5);  
        // Set the point representation
        reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

        reg.setInputCloud (points_with_normals_src);
        reg.setInputTarget (points_with_normals_tgt);



        //
        // Run the same optimization in a loop and visualize the results
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
        PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
        reg.setMaximumIterations (100);
        for (int i = 0; i < 30; ++i)
        {
          PCL_INFO ("Iteration Nr. %d.\n", i);

          // save cloud for visualization purpose
          points_with_normals_src = reg_result;

          // Estimate
          reg.setInputCloud (points_with_normals_src);
          reg.align (*reg_result);

                      //accumulate transformation between each Iteration
          Ti = reg.getFinalTransformation () * Ti;

                      //if the difference between this transformation and the previous one
                      //is smaller than the threshold, refine the process by reducing
                      //the maximal correspondence distance
          if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

          prev = reg.getLastIncrementalTransformation ();

          // visualize current state
  //        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
        }

              //
        // Get the transformation from target to source
        targetToSource = Ti.inverse();
        targetToSource = targetToSource * generateTransformMatrixY(7.2);

        //
        // Transform target back in source frame
        pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  //      p->removePointCloud ("source");
  //      p->removePointCloud ("target");
  //
  //      PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  //      PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  //      p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  //      p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

  //            PCL_INFO ("Press q to continue the registration.\n");
  //      p->spin ();
  //
  //      p->removePointCloud ("source"); 
  //      p->removePointCloud ("target");

        //add the source to the transformed target
        *output += *cloud_src;

        final_transform = targetToSource;
     }
};


#endif	/* PCLOPERATIONS_H */

