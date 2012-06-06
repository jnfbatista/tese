#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

#include <boost/asio.hpp>

#include "PCLWorker/PCLWorker.h"
#include "Message/Message.h"
#include "MessageServer/MessageServer.h"
#include "gui/Gui.h"

void usage() {
	printf("Usage of the application:\n");
	printf("-c\tCapture and analyze point clouds in realtime\n");
	printf("-v\tShow a point cloud\n");
}


int main ( int argc, char* argv[], char* envp[]) {
	if (argc > 1) {
		if(strcmp(argv[1], "-c") == 0 && argc > 3) {	
			PCLWorker v;
			v.run ( argv[2], argv[3]);
		} else if (strcmp(argv[1], "-g") == 0) {
			printf("Hopefully a GUI will appear...\n");
			Gui gui = Gui(argc, argv);
		} else if (strcmp(argv[1], "-m") == 0 ) {
			Message msg = Message();
			msg.test_message();
		} else if (strcmp(argv[1], "-s") == 0 ) {
			PCLWorker v;
			v.run_capture(30);
		} else if (strcmp(argv[1], "-p") == 0) { 
			printf("Processing a image\n");
			PCLWorker p;
			p.analyze_image(argv[2]);
		} else if (strcmp(argv[1], "-v") == 0) {
			if (argc == 3) {
				// argument char* to string
				std::string fileName = std::string(argv[2]);

				PCLWorker pcl = PCLWorker();

				pcl.display_point_cloud(fileName);

			} else {
				printf("Missing the file path\n");
			}
		} else if (strcmp(argv[1], "-us") == 0 && argc == 3) {
			try
			{
				printf("UDP - started serving.\n");

				boost::asio::io_service io_service;
				MessageServer* server = new MessageServer(io_service, atoi(argv[2]));
				io_service.run();
				printf("UDP - stopped serving.\n");

			}
			catch (std::exception& e)
			{
				std::cerr << e.what() << std::endl;
			}

		} else if (strcmp(argv[1], "-uc") == 0) {
			try {

				Message *msg = new Message();

				vector<string> msg_param;
				msg_param.push_back("distancia");
				msg_param.push_back("8,54");

				msg->add_param(msg_param);


				msg->send_message(argv[2], "8888");

			}
			catch (std::exception& e)
			{
				std::cerr << e.what() << std::endl;
			}
		} else if (strcmp(argv[1], "-bg") == 0) {
			//PCLWorker p;

			//p.calibrate_background(argv[2], argv[3], argv[4]);

		}


	} else {
		// TODO Add a more detailed rundown of the commands that can be provided
		printf("No options provided!\n");

	}
	return 0;
}
