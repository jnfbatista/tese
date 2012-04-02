#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

class PCLWorker {

	public:
	
		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
		void run (); 

		bool calibrate_background();

};


