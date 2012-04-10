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

void PCLWorker::run_capture() {

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

void PCLWorker::display_point_cloud(string filename) {
	// show it!!
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	// load image
	printf("loading image...\n");
	pcl::io::loadPCDFile(filename, cloud);
	printf("pcd file loaded\n");

	// Initiates visualizer
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr ptr_cloud = cloud.makeShared();
	viewer.showCloud(ptr_cloud, "teste");
	while (!viewer.wasStopped())
	{
		sleep(1);
	}

}

void PCLWorker::run (string address, string port){
	// create a new grabber for OpenNI devices
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

//	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
//		boost::bind (&PCLWorker::cloud_cb_, this, _1);
	// make callback function from member function
	boost::function<void ( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind (&PCLWorker::calibrate_background, this, _1, address, port);

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

void PCLWorker::calibrate_background(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_o, string address, string port) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>());

	copyPointCloud(*cloud_o, *cloud);

	// load image
	/*printf("loading image from file...\n");
	pcl::io::loadPCDFile(filename, *cloud);
	printf("pcd file loaded\n");
*/
	if (cloud->isOrganized()) {
		printf("Cloud is organized!\n");
	} else {
		printf("Cloud is NOT organized!\n");
	}


	// Rotate the point cloud 
	printf("Rotating the cloud\n");
	// defines the rotation matrix
	Eigen::AngleAxis<float> aa(M_PI / 180 * -90, Eigen::Vector3f::UnitZ());
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(aa);
	pcl::transformPointCloud(*cloud, *cloud, transform);


	// remove after a given distance
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.70, 3.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud);

	// Remove outliers
	/*pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
	// build the filter
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.8);
	outrem.setMinNeighborsInRadius (100);
	// apply filter
	outrem.filter (*cloud);
	*/

	pcl::copyPointCloud(*cloud, *cloud_points);


	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Optional
	seg.setOptimizeCoefficients (true);

	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);

	// @TODO remove the magic number
	seg.setDistanceThreshold (0.08);


	seg.setInputCloud (cloud_points->makeShared ());
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return ;
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " " 
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

	printf("Starting removal of planar points...\n", cloud->size());


	// Indices extractor object
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	// Extract the inliers
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud);

	printf("End of removal of planar points...\n", cloud->size());


	// Diferentiate objects
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance (0.1); // 50cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (cloud->points.size() *0.80);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	pcl::PCDWriter writer;


	std::vector<pcl::PointCloud<pcl::PointXYZRGBA> > clusters;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_cluster;// (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster.points.push_back (cloud->points[*pit]); //*

		cloud_cluster.width = cloud_cluster.points.size ();
		cloud_cluster.height = 1;
		cloud_cluster.is_dense = true;


		// puts it in the vector
		clusters.push_back(cloud_cluster);

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster.points.size () << " data points." << std::endl;
		//std::stringstream ss;
		//ss << "cloud_cluster_" << j << ".pcd";
		//writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_cluster, false); //*
		j++;
	}

	// calculate the distance to each
	for (int i = 0; i < clusters.size(); i++) {
		float x = 0.0, y = 0.0, z = 0.0, radius = 0.0;

		for(int p = 0; p < clusters[i].size(); p++) {
			x += clusters[i].points[p].x;
			y += clusters[i].points[p].y;
			z += clusters[i].points[p].z;
		}

		float x_med = x / clusters[i].size();
		float y_med = y / clusters[i].width;
		float z_med = z / clusters[i].width;

		printf("\nmedia do cluster %i encontra-se em:\n x:\t%f\t%f,\ny:\t%f\t%f,\nz:\t%f\t%f\n", i,x, x_med,y, y_med,z, z_med);

		Message m;
		m.add_param("centro_x",  boost::lexical_cast<string>( x_med ));
		m.add_param("centro_y",  boost::lexical_cast<string>( y_med ));
		m.send_message(address, port);
	}




	// create message
	// send by udp
	// go play something

	// Initiates visualizer
	/*
	   pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	   const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr ptr_cloud = cloud->makeShared();
	   viewer.showCloud(ptr_cloud, "teste");
	   while (!viewer.wasStopped())
	   {
	   sleep(1);
	   }
	   */

}
