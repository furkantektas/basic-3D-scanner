/* 
 * File:   PCDIO.hpp
 * Author: furkan
 *
 */

#ifndef PCDIO_HPP
#define	PCDIO_HPP


//#include <iostream>
#include "types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

    class PCDIO {
    public:
        
        static bool load(const std::string &file_name, ColorCloud& cloud){
            if (pcl::io::loadPCDFile<PointT> (file_name, cloud) == -1) {
                PCL_ERROR("Couldn't read file \n");
                return false;
            }
            return true;
        }
        
        static bool save(const std::string &file_name, const Cloud& cloud){
            if (pcl::io::savePCDFile<PointT> (file_name, cloud) == -1) {
                PCL_ERROR("Couldn't write to file \n");
                return false;
            }
            return true;
        }

    };

#endif	/* PCDIO_HPP */

