#include "PCLWorker.h"
void PCLWorker::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {

	static unsigned count = 0;
	static double last = pcl::getTime ();

	// if it counts 30 'frames'
	if (++count == 30) {
		double now = pcl::getTime ();
		std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
		count = 0;
		last = now;

		// Saves the PCF file
		pcl::io::savePCDFileASCII (boost::lexical_cast<std::string>(now), *cloud);
		std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
	}

}

void PCLWorker::run() {

	// create a new grabber for OpenNI devices
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	// make callback function from member function
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind (&PCLWorker::cloud_cb_, this, _1);

	// connect callback function for desired signal. In this case its a point cloud with color values
	boost::signals2::connection c = interface->registerCallback (f);

	// start receiving point clouds
	interface->start ();

	// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
	while (true)
		sleep(1);

	// stop the grabber
	interface->stop ();
}
