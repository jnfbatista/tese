#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// segmentation related includes
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/timer.hpp> 

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <map>
#include <cmath>
#include <string>
#include <time.h>
#include <exception>

#include "../Message/Message.h"
#include "../ModelDictionary/ModelDictionary.h"
#include "../Model/Model.h"
/*
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "processfunc.h"


*/

#define PI 3.14159265

using std::string;

/**
 * This class interfaces with the Kinect device (or other PCL supporte IO device) and processes the
 * input.
 */
class PCLWorker {

	public:

		/**
		 * This funcion is used to capture new pcd files
		 */
		void run_capture (int); 

		/**
		 * Captures the point cloud to a file
		 */
		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, int);
		void cloud_cb_v (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

		void pcl_viewer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ); 

		// displays a point saved point cloud file 
		void display_point_cloud(string);

		/**
		 * This function runs the recognition process
		 */
		void run (string,string); 

		/**
		 * Processes a point cloud to find predetermined objects
		 */
		void process(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, string, string);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pre_process(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, string, string);
		void analyze_image(string, ModelDictionary);


		// Returns the found clusters
		std::vector<pcl::PointCloud<pcl::PointXYZ> > detect_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

		void paint_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointCloud<pcl::PointXYZ> >);

		void cluster_morfology(pcl::PointCloud<pcl::PointXYZ>, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f );

		/*
		 * This is where the magic happens
		 */
		std::vector<pcl::PointCloud<pcl::PointXYZ> > find_table(pcl::PointCloud<pcl::PointXYZ>::Ptr, bool, ModelDictionary);

		void concat_clouds(string,string);

		// line processing sheananigans
		//void process_rgb(cv::Mat);

		time_t seconds;


};


