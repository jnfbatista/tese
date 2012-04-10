#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string.h>

// segmentation related includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <boost/lexical_cast.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include "../Message/Message.h"

using std::string;

/**
 * This class interfaces with the Kinect device (or other PCL supporte IO device) and processes the
 * input.
 */
class PCLWorker {

	public:
	
		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

		void run (string,string); 
		void run_capture (); 

		//displays a point saved point cloud file 
		void display_point_cloud(string);

		void calibrate_background(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, string, string);

};


