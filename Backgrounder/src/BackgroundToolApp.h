#pragma once

#include "ofMain.h"
#include "ofxSimpleGuiToo.h"
#include "BackgroundCalculator.h"

class BackgroundToolApp : public ofBaseApp{

	public:

		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);		


		int backgroundMethod;
		int lastBackgroundMethod;

		ofVideoPlayer 		fingerMovie;
		unsigned int inputFrameCount;

		unsigned int currentFrame;

		bool                frameByframe;

		bool dragging;

		ofxSimpleGuiMovieInOut* inOutControl;

		ofImage background;

		float backgroundPercent;
		unsigned int lastFrame;

		ofxSimpleGuiContent* backgroundDisplay;


		BackgroundCalculator* backgroundCalculator;
		float lastIn;
		float lastOut;
		bool backgroundIsStale;
		bool backgroundIsUpdating;
		int inFrame;
		int outFrame;

		bool saveBackground;
		bool loadVideo;

		std::string inFilename;

		//ofxSimpleGuiMovieInOut inOutControl;
};

