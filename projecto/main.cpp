#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <vector>
#include <pthread.h>

#include "KinectDevice/KinectDevice.cpp"

using namespace std;

//define OpenGL variables
GLuint gl_depth_tex;
GLuint gl_rgb_tex;
int g_argc;
char **g_argv;
int got_frames(0);
int window(0);

//define libfreenect variables
Freenect::Freenect freenect;
KinectDevice* device;
double freenect_angle(0);
freenect_video_format requested_format(FREENECT_VIDEO_RGB);


//define Kinect Device control elements
//glutKeyboardFunc Handler
void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		device->setLed(LED_OFF);
		freenect_angle = 0;
		glutDestroyWindow(window);
	}
	if (key == '1') {
		device->setLed(LED_GREEN);
	}
	if (key == '2') {
		device->setLed(LED_RED);
	}
	if (key == '3') {
		device->setLed(LED_YELLOW);
	}
	if (key == '4') {
		device->setLed(LED_BLINK_GREEN);
	}
	if (key == '5') {
		// 5 is the same as 4
		device->setLed(LED_BLINK_GREEN);
	}
	if (key == '6') {
		device->setLed(LED_BLINK_RED_YELLOW);
	}
	if (key == '0') {
		device->setLed(LED_OFF);
	}
	if (key == 'f') {
		if (requested_format == FREENECT_VIDEO_IR_8BIT) {
			requested_format = FREENECT_VIDEO_RGB;
		} else if (requested_format == FREENECT_VIDEO_RGB){
			requested_format = FREENECT_VIDEO_YUV_RGB;
		} else {
			requested_format = FREENECT_VIDEO_IR_8BIT;
		}
		device->setVideoFormat(requested_format);
	}

	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	}
	if (key == 's' || key == 'd') {
		freenect_angle = 10;
	}
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	}
	if (key == 'e') {
		freenect_angle = 10;
	}
	if (key == 'c') {
		freenect_angle = -10;
	}
	device->setTiltDegrees(freenect_angle);
}
//define OpenGL functions
void DrawGLScene()
{
	static std::vector<uint8_t> depth(640*480*4);
	static std::vector<uint8_t> rgb(640*480*4);

	// using getTiltDegs() in a closed loop is unstable
	/*if(device->getState().m_code == TILT_STATUS_STOPPED){
		freenect_angle = device->getState().getTiltDegs();
	}*/
	device->updateState();
	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
	fflush(stdout);

	device->getDepth(depth);
	device->getRGB(rgb);

	got_frames = 0;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 4, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(640,480,0);
	glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();
	glutSwapBuffers();
}

void InitGL()
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 640, 480, 0, 0.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void displayKinectData(KinectDevice* device){
	glutInit(&g_argc, g_argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	window = glutCreateWindow("c++ wrapper example");
	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
	glutKeyboardFunc(&keyPressed);
	InitGL();
	glutMainLoop();
}
//define main function
int main(int argc, char **argv) {
	//Get Kinect Device
	device = &freenect.createDevice<KinectDevice>(0);
	//Start Kinect Device
	device->setTiltDegrees(10);
	//device->startVideo();
	device->startDepth();
	//handle Kinect Device Data
	device->setLed(LED_RED);
	displayKinectData(device);
	//Stop Kinect Device
	//device->stopVideo();
	device->stopDepth();
	device->setLed(LED_OFF);
	return 0;
}

//int main(int argc, char * argv[], char * envp[]) {
//
//	// The thread which gets the kinect feed will be here
//	
//	// the thread to process/recognize it will be here
//
//	printf("Hello world!\n");
//	return 0;
//}
