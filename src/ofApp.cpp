#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	ofSetFrameRate(30); // kinect's fps is 30fps
	setup_kinect();

	//fudeTracker.setup(depthWidth, depthHeight);

	setup_gui();

	display_mode = MODE_CALIBRATION_DEPTH;

	setup_osc();

}

//--------------------------------------------------------------
void ofApp::update(){

	// change to tracking phase
	if (phase == PHASE_SET_BG &&
		ofGetElapsedTimeMillis() - start_bg_calc_time > BG_PHASE_DURATION)
	{
		phase = PHASE_CALC;
	}

	if (!kinect.grabNewFrame()) return;

	// grab depth image
	depthMat16 = kinect.getDepthData16();

	//switch (phase)
	//{
	//case PHASE_SET_BG:
	//	ballFinder.calcBackground(depthMat16);
	//	break;
	//case PHASE_CALC:
	//	ballFinder.calcFrontImage(depthMat16);
	//	ballFinder.findBalls();
	//	break;
	//default:
	//	break;
	//}

	if (display_mode == MODE_CALIBRATION_COLOR)
	{
		kinect.grabColorFrame();

		// grab colored depth image
		coloredDepthMat = kinect.getColoredDepthMat();
	}

	// update kinect image
	kinect.grabInfraredFrame();

	// grab color image
	colorMat = kinect.getColorMat();

	// grab mask image
	//ballMat = ballFinder.getBallMat();


	// set ball roi in depth and color coordinate
	//set_color_ball_rects(ballTracker.getCandidates());
	//set_depth_ball_rects(ballTracker.getCandidates());

	// send data
	//send_data(dummyBalls.getBalls());

	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofBackground(0);

	switch (display_mode)
	{
	case MODE_CALIBRATION_DEPTH:
		draw_calibration_depth();
		break;

	case MODE_CALIBRATION_COLOR:
		draw_calibration_color();
		break;

	case MODE_TRACKING:
		draw_tracking();
		break;

	default:
		break;
	}
}

//--------------------------------------------------------------
void ofApp::exit() {

	kinect.exit();
	maskGenerator.savePointData();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

	switch (key)
	{
	case '1':
		display_mode = MODE_CALIBRATION_DEPTH;
		break;
	case '2':
		display_mode = MODE_CALIBRATION_COLOR;
		break;
	case '3':
		display_mode = MODE_TRACKING;
		break;
	default:
		break;
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ofApp::setup_gui()
{
	//kinect_params.setName("kinect params");
	//kinect_params.add(bShowMask.set("show mask", true));

	//// gui setup
	//kinect_params.add(canny1.set("canny1", 50, 0, 255));
	//kinect_params.add(canny2.set("canny2", 50, 0, 255));
	//kinect_params.add(ball_radius_min.set("ball_radius_min", 1, 1, 100));
	//kinect_params.add(ball_radius_max.set("ball_radius_max", 20, 1, 100));
	//kinect_params.add(erode_iteration.set("erode", 1, 1, 10));
	//kinect_params.add(dilate_iteration.set("dilate", 1, 1, 10));
	//kinect_params.add(blur_iteration.set("blur", 3, 1, 21));
	//kinect_params.add(erode_edge.set("erode edge", 1, 0, 10));
	//kinect_params.add(dilate_edge.set("dilate edge", 1, 0, 10));
	//kinect_params.add(blur_edge.set("blur edge", 3, 0, 21));
	//kinect_params.add(min_age.set("min_age", 10, 1, 100));


	//gui_kinect.setup(kinect_params, "kinect_settings.xml");
	//gui_kinect.loadFromFile("kinect_settings.xml");
	//gui_kinect.setPosition(ofGetWindowWidth() - MY_GUI_SIZE, 10);

	params.setName("params");
	params.add(bShowMask.set("show gui", true));
	gui.setup(params, "settings.xml");

	showGui = true;

}

//--------------------------------------------------------------
void ofApp::setup_kinect()
{
	if (!kinect.setup())
	{
		exit();
		return;
	}

	depthWidth = kinect.getWidth();
	depthHeight = kinect.getHeight();
	cout << "depth w: " << depthWidth << " h: " << depthHeight << endl;

	colorWidth = kinect.getColorWidth();
	colorHeight = kinect.getColorHeight();
	cout << "color w: " << colorWidth << " h: " << colorHeight << endl;

	// allocate Mat
	depthMat16 = cv::Mat(depthHeight, depthWidth, CV_16SC1);
	depthMat = cv::Mat(depthHeight, depthWidth, CV_8SC1);
	irMat = cv::Mat(depthHeight, depthWidth, CV_8SC1);
	colorMat = cv::Mat(colorHeight, colorWidth, CV_16SC3);
	coloredDepthMat = cv::Mat(depthHeight, depthWidth, CV_8SC3);

	// マスク画像生成
	maskGenerator.setup(depthWidth, depthHeight, "Homography.xml");

	ofImage tmpImage;
	tmpImage.setFromPixels(maskGenerator.getMaskPixels(), depthWidth, depthHeight, OF_IMAGE_COLOR_ALPHA);
	fudeFinder.setMaskImage(tmpImage);

	// homographyクラス初期化
	updateHomography();

	start_calc_bg();
}

//--------------------------------------------------------------
void ofApp::updateHomography()
{
	vector<cv::Point2f> srcPoints, dstPoints;

	ofVec2f* maskPoints = maskGenerator.getHomographyPoints();
	srcPoints.push_back(cv::Point2f(maskPoints[0].x, maskPoints[0].y));
	srcPoints.push_back(cv::Point2f(maskPoints[1].x, maskPoints[1].y));
	srcPoints.push_back(cv::Point2f(maskPoints[2].x, maskPoints[2].y));
	srcPoints.push_back(cv::Point2f(maskPoints[3].x, maskPoints[3].y));

	dstPoints.push_back(cv::Point2f(0, 0));
	dstPoints.push_back(cv::Point2f(1, 0));
	dstPoints.push_back(cv::Point2f(1, 1));
	dstPoints.push_back(cv::Point2f(0, 1));
	homography.calcHomography(srcPoints, dstPoints);
}

//--------------------------------------------------------------
void ofApp::start_calc_bg()
{
	fudeFinder.resetBackground();
	phase = PHASE_SET_BG;
	start_bg_calc_time = ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::setup_osc()
{
	// OSC
	ofXml XML;
	if (XML.load("osc_settings.xml")) {
		cout << "mySettings.xml loaded!" << endl;
	}

	if (XML.exists("//IP")) {
		targetIP = XML.getValue<string>("//IP");
	}
	else {
		targetIP = "localhost";
	}

	if (XML.exists("//PORT")) {
		targetPort = XML.getValue<int>("//PORT");
	}
	else {
		targetPort = 20001;
	}

	if (XML.exists("//ADDRESS")) {
		oscAddress = "/" + XML.getValue<string>("//ADDRESS");
	}
	else {
		oscAddress = "/kinect1";
	}

	sender.setup(targetIP, targetPort); // open an outgoing connection to HOST:PORT
}

////--------------------------------------------------------------
//void ofApp::send_data(vector<BocciaBall> balls)
//{
//	ofxOscMessage m;
//
//	for each (auto ball in balls)
//	{
//		// coordinate transformation. depth -> color -> homography
//		ofPoint tmpDepthPos = homography_color.getTransformedPoint(kinect.depthToColor(ball.get_depth_pos()));
//
//		// scale and add offset
//		ofVec2f depthPos = adjust_position(tmpDepthPos);
//
//		// coordinate transformation. color -> homography
//		ofPoint tmpColorPos = homography_color.getTransformedPoint(ball.get_color_pos());
//
//		// scale and add offset
//		ofVec2f colorPos = adjust_position(tmpColorPos);
//
//		// no color data
//		if (ball.isDummy)
//		{
//			depthPos = colorPos;
//			//colorPos = depthPos;
//		}
//
//		//cout << depthPos << endl;
//		m.clear();
//		m.setAddress(oscAddress);	// xmlで設定する
//		m.addIntArg(ball.ID);
//		m.addFloatArg(depthPos.x);
//		m.addFloatArg(depthPos.y);
//		m.addFloatArg(colorPos.x);
//		m.addFloatArg(colorPos.y);
//		m.addStringArg(ball.get_color());
//		m.addIntArg(ball.age);
//		sender.sendMessage(m, false);
//		//cout << output.x << " " << output.y << endl;
//	}
//
//	m.clear();
//	m.setAddress(oscAddress + "update");	// xmlで設定する
//	m.addIntArg(1);
//	sender.sendMessage(m, false);
//}

//--------------------------------------------------------------
void ofApp::draw_calibration_depth()
{
	kinect.drawDepthImage(0, 0, depthWidth, depthHeight);
	kinect.drawInfraredImage(depthWidth, 0, depthWidth, depthHeight);

	//ofSetColor(255, 255, 255);
	////gui_kinect.draw();
	//if (showGui)
	//{
	//	gui_ball_find.draw();
	//	gui_ball_tracker.draw();
	//	gui_ball_color.draw();
	//}
}

//--------------------------------------------------------------
void ofApp::draw_calibration_color()
{
	kinect.drawColoredDepthImage(0, 0, depthWidth, depthHeight);

	// vertices for homography
	if (bShowMask)
	{
		ofPushStyle();
		ofSetColor(64, 32);
		maskGenerator.draw_mask(0, 0);
		ofPopStyle();
	}

	//ofSetColor(255, 255, 255);
	////gui_kinect.draw();
	//if (showGui)
	//{
	//	gui_ball_find.draw();
	//	gui_ball_tracker.draw();
	//	gui_ball_color.draw();
	//}
}

//--------------------------------------------------------------
void ofApp::draw_tracking()
{
	kinect.drawColoredDepthImage(0, 0, depthWidth, depthHeight);

	ofSetColor(255);
	//ofDrawBitmapString("Tracking mode", 10, ofGetWindowHeight() - 20);
	if (showGui)
	{
		//gui_support.draw();
	}

}