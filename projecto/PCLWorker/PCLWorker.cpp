#include "PCLWorker.h"
void PCLWorker::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, int frame_gap) {

	static unsigned count = 0;
	static double last = pcl::getTime ();

	// if it counts 30 'frames'
	if (++count == frame_gap) {
		double now = pcl::getTime ();
		std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
		count = 0;
		last = now;

		// Saves the PCF file
		pcl::io::savePCDFileASCII (boost::lexical_cast<std::string>(now) + ".pcd" , *cloud);
		std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
	}

}

void PCLWorker::run_capture(int frame_gap) {

	// create a new grabber for OpenNI devices
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	// make callback function from member function
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind (&PCLWorker::cloud_cb_, this, _1, frame_gap);

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
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	// load image
	printf("loading image...\n");
	pcl::io::loadPCDFile(filename, cloud);
	printf("pcd file loaded\n");

	// Rotate the point cloud 
	//	printf("Rotating the cloud\n");
	//	// defines the rotation matrix
	//	Eigen::AngleAxis<float> aa(M_PI / 180 * -90, Eigen::Vector3f::UnitZ());
	//	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//	transform.rotate(aa);
	//	pcl::transformPointCloud(cloud, cloud, transform);


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
		boost::bind (&PCLWorker::process, this, _1, address, port);

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


void PCLWorker::process(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_o, string address, string port) {

	static unsigned count = 0;
	static double last = pcl::getTime ();

	// chamar as funções do rodolfo
	//      static cv::Mat m(cv::Size(cloud_o->width,cloud_o->height), CV_8UC3, cv::Scalar(0,0,0));
	//	static	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_r (new pcl::PointCloud<pcl::PointXYZRGBA>());
	//	copyPointCloud(*cloud_o, *cloud_r);

	//      printf("size: w%i h%i\nfirst element size: %i\n", cloud_o->width,cloud_o->height, cloud_o->points.size());

	//	cv::MatIterator_<cv::Vec3b> it, end;
	//	vector<vector<float> > depth;
	//
	//	int p  = 0;
	//	vector<float> temp;
	//	for( it = m.begin<cv::Vec3b>(), end = m.end<cv::Vec3b>(); it != end; ++it) {
	//		(*it)[0] = cloud_r->points[p].b;
	//		(*it)[1] = cloud_r->points[p].g;
	//		(*it)[2] = cloud_r->points[p].r; 
	//
	//		// para a passagem do depth 
	//		if (p % cloud_r->width == 0  && p > 0 ) {
	//			depth.push_back(temp);
	//			temp.clear();
	//			temp.push_back(cloud_r->points[p].z);
	//			//printf("fim de linha (ponto %i) \n", p);
	//		} else {
	//			temp.push_back(cloud_r->points[p].x);
	//		}
	//
	//		p++;
	//	}
	//	
	//	//cv::imwrite("teste.png", m);
	//	
	//	
	//	if (!imageProcessing(m, depth)) {
	//		printf("O rodolfo fez asneira\n");
	//	}

	// if it counts 30 'frames'
	if (++count == 12) {
		pre_process(cloud_o, address, port);
		// zero frames
		count = 0;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCLWorker::pre_process(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_o, string address, string port) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>());

	copyPointCloud(*cloud_o, *cloud);

	//	// Rotate the point cloud 
	//	printf("Rotating the cloud\n");
	//	// defines the rotation matrix
	//	Eigen::AngleAxis<float> aa(M_PI / 180 * -90, Eigen::Vector3f::UnitZ());
	//	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//	transform.rotate(aa);
	//	pcl::transformPointCloud(*cloud, *cloud, transform);


	printf("filtering distances... ");
	// remove after a given distance
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.70, 3.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud);

	printf("done !\n");


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

	if (inliers->indices.size () != 0)
	{
		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " " 
			<< coefficients->values[3] << std::endl;

		std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

		printf("Starting removal of planar points... ", cloud->size());


		// Indices extractor object
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		// Extract the inliers
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter (*cloud);

		printf("done!\n", cloud->size());


	} else {

		printf("No plane for you dude!\n");
		/*	sleep(1);
			PCL_ERROR ("Could not estimate a planar model for the given dataset.");
			return ;*/
	}


	// Diferentiate objects using clusters
	// Creating the KdTree object for the search method of the extraction
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud (cloud);
//
//	std::vector<pcl::PointIndices> cluster_indices;
//	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//	ec.setClusterTolerance (0.1); // 50cm
//	ec.setMinClusterSize (100);
//	ec.setMaxClusterSize (cloud->points.size() );
//	ec.setSearchMethod (tree);
//	ec.setInputCloud (cloud);
//	ec.extract (cluster_indices);
//
//
//	std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;
//
//	int j = 0;
//	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//	{
//		pcl::PointCloud<pcl::PointXYZ> cloud_cluster;// (new pcl::PointCloud<pcl::PointXYZRGBA>);
//		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
//			cloud_cluster.points.push_back (cloud->points[*pit]); //*
//
//		cloud_cluster.width = cloud_cluster.points.size ();
//		cloud_cluster.height = 1;
//		cloud_cluster.is_dense = true;
//
//
//		// puts it in the vector
//		clusters.push_back(cloud_cluster);
//
//		std::cout << "PointCloud representing the Cluster: " << cloud_cluster.points.size () << " data points." << std::endl;
//		//std::stringstream ss;
//		//ss << "cloud_cluster_" << j << ".pcd";
//		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//		j++;
//	}
//
//	if(clusters.size() == 0) {
//		printf("No cluster found\n");
//	}
//
	// calculate the distance to each
	/*	for (int i = 0; i < clusters.size(); i++) {
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

	// create message
	// send by udp
	Message m;
	m.add_param("centro_x",  boost::lexical_cast<string>( z_med ));
	m.add_param("centro_y",  boost::lexical_cast<string>( x_med ));
	m.send_message(address, port);
	}*/

	return cloud;

}

void PCLWorker::pcl_viewer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud) {
	// Initiates visualizer
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(ptr_cloud, "teste");

	while (!viewer.wasStopped())
	{
		sleep(1);
	}
}


std::vector<pcl::PointCloud<pcl::PointXYZ> > PCLWorker::detect_clusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	
	// Looks for clusters representing the table
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.5); // 50cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (cloud->points.size() );
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);


	std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_cluster;// (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}

	if(clusters.size() == 0) {
		printf("No cluster found\n");
	}

	return clusters;
}


void PCLWorker::analyze_image(string file_path) {

	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

	// load image
	printf("loading image...\n");
	pcl::io::loadPCDFile(file_path, cloud);
	printf("pcd file loaded\n");

	// pre processing
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_res = pre_process(cloud.makeShared(), "","");

	std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters = detect_clusters(cloud_res);

	for (int i = 0; i < clusters.size(); i++) {
		// finds a table
		printf("Analyzing cluster number %d\n", i +1 );
		find_table(clusters[i].makeShared(), false);
		pcl_viewer(clusters[i].makeShared());
		printf("Done.\n");
	}

	// finds a table
	//find_table(cloud_res, true);

	//const pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr_cloud = cloud_res->makeShared();

	//pcl_viewer(ptr_cloud);
}


void PCLWorker::find_table(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool remove_plane) {

	bool has_plane = false;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::copyPointCloud(*cloud, *cloud_points);

	// First step, find plane (tabletop)
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

	if (inliers->indices.size () != 0)
	{
		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " " 
			<< coefficients->values[3] << std::endl;

		std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

		has_plane = true;

		if (remove_plane) {
			printf("Starting removal of planar points... ", cloud->size());

			// Indices extractor object
			pcl::ExtractIndices<pcl::PointXYZ> extract;

			// Extract the inliers
			extract.setInputCloud (cloud);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud);

			printf("done!\n", cloud->size());
		}

	} else {
		printf("No plane for you dude!\n");
	}


	// Looks for clusters representing the table
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.1); // 50cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (cloud->points.size() );
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);


	std::vector<pcl::PointCloud<pcl::PointXYZ> > clusters;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_cluster;// (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}

	if(clusters.size() == 0) {
		printf("No cluster found\n");
	} else {
		printf("Number of clusters found: %d.\n", clusters.size());
	}


	// Prints the probability of being a table
	float probability = clusters.size() / 5.0;

	if (has_plane )
		probability += 0.2;

	printf("Probability of a table: %f\n", probability );


}
