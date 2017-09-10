#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"
#include "ofxOsc.h"

#include "ofxKinectDepth16.h"
#include "MaskGenerator.h"
#include "HomographyTransform.h"
#include "FudeFinder.h"

enum MY_TRACKING_PHASE
{
	PHASE_SET_BG,
	PHASE_CALC,
};

enum MY_DISPLAY_MODE
{
	MODE_CALIBRATION_DEPTH,
	MODE_CALIBRATION_COLOR,
	MODE_DUMMY,
	MODE_TRACKING,
};

// constants
static const int BG_PHASE_DURATION = 2000;

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
private:
	// utilities
	void setup_gui();
	void setup_kinect();
	void setup_osc();
	void updateHomography();
	void start_calc_bg();
	//void send_data(vector<BocciaBall> balls);

	// draws
	void draw_calibration_depth();
	void draw_calibration_color();
	//void draw_dummy();
	void draw_tracking();

	// kinect
	ofxKinectDepth16 kinect;
	int depthWidth, depthHeight;
	int colorWidth, colorHeight;

	// for fude tracking
	FudeFinder fudeFinder;
	cv::Mat depthMat16;
	cv::Mat depthMat;
	cv::Mat colorMat;
	cv::Mat coloredDepthMat;
	cv::Mat irMat;

	// for coordinate settings
	CMaskGenerator maskGenerator;
	CHomographyTransform homography;

	// phase control
	MY_TRACKING_PHASE phase;
	unsigned long long start_bg_calc_time;
	MY_DISPLAY_MODE display_mode;

	//gui settings--------------------
	ofxPanel gui;
	ofParameterGroup params;
	bool showGui;
	ofParameter<bool> bShowMask;

	//OSC params------------------
	string targetIP;
	int targetPort;
	string oscAddress;
	ofxOscSender sender;
};
