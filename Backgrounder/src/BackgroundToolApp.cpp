#include "BackgroundToolApp.h"
#include "BackgroundCalculatorAverage.h"
#include "BackgroundCalculatorMode.h"

//--------------------------------------------------------------
void BackgroundToolApp::setup(){
	ofBackground(0,0,0);

	backgroundCalculator = NULL;

	frameByframe = false;

	dragging = false;

	backgroundPercent = 0.0;

	gui.setup();
	gui.config->gridSize = ofPoint(512, 512);
	gui.config->offset.y = -92;
	gui.config->offset.x = 2;
	
	ofxSimpleGuiConfig* progressConfig = new ofxSimpleGuiConfig();
	progressConfig->fullColor = 0x66CC66;

	ofxSimpleGuiButton* loadButton = new ofxSimpleGuiButton("Load Video...", loadVideo);
	loadButton->setToggleMode(true);
	loadButton->setNewColumn(false);
	gui.addControl(*loadButton);

	inOutControl = new ofxSimpleGuiMovieInOut("In and Out", &fingerMovie);
	gui.addControl(*inOutControl);

	string backgroundMethods[] = {"Average", "Mode", "Median"};
	gui.addComboBox("Background Method", backgroundMethod, 3, backgroundMethods);

	ofxSimpleGuiButton* saveButton = new ofxSimpleGuiButton("Save Background...", saveBackground);
	saveButton->setToggleMode(true);
	saveButton->setNewColumn(true);
	gui.addControl(*saveButton);

	
	backgroundDisplay = new ofxSimpleGuiContent("Background", background, 512);
	backgroundDisplay->setNewColumn(false);
	gui.addControl(*backgroundDisplay);


	ofxSimpleGuiSliderFloat* backgroundProgressSlider = new ofxSimpleGuiSliderFloat("Progress", backgroundPercent, 0.0, 100.0);
	backgroundProgressSlider->disableAllEvents();
	//backgroundProgressSlider->config->fullActiveColor = 0x66CC66;
	backgroundProgressSlider->setConfig(progressConfig);
	gui.addControl(*backgroundProgressSlider);

	
	saveBackground = false;
	loadVideo = true;
	
	backgroundIsStale = true;
	backgroundIsUpdating = false;

	//gui.loadFromXML();
	gui.show();
}

//--------------------------------------------------------------
void BackgroundToolApp::update(){
    fingerMovie.idleMovie();

	if(saveBackground)
	{
		ofFileDialogResult result = ofSystemSaveDialog( inFilename + "-background.png", "Save background image"); // TODO go into oF and enforce image format and fix default filename
		ofImage charBackground = backgroundCalculator->currentBackground;
		
		ofSaveImage(charBackground, result.getPath() + ".png", OF_IMAGE_QUALITY_BEST);
		
		saveBackground = false;
	}

	if(loadVideo)
	{
		ofFileDialogResult result = ofSystemLoadDialog("Load video");
		inFilename = result.getName();

		ofVideoPlayer newMovie;
		if(result.bSuccess && newMovie.loadMovie(result.getPath()))
		{
			fingerMovie = newMovie;
			fingerMovie.setPaused(true);

			inOutControl->setVideo(&fingerMovie);
			inputFrameCount = fingerMovie.getTotalNumFrames();
			//exit(0);
		}
		else
		{
			if(result.bSuccess)
				ofSystemAlertDialog(result.filePath + " could not be loaded. The file is an unsupported format or is corrupt.");
		}
		loadVideo = false;
	}

	if( !dragging && (backgroundMethod != lastBackgroundMethod || lastIn != inOutControl->in || lastOut != inOutControl->out) )
	{
		if(backgroundCalculator != NULL)
			delete(backgroundCalculator);

		if(backgroundMethod == BACKGROUND_METHOD_AVERAGE)
			backgroundCalculator = new BackgroundCalculatorAverage(&fingerMovie, inOutControl->in, inOutControl->out);
		if(backgroundMethod == BACKGROUND_METHOD_MODE)
			backgroundCalculator = new BackgroundCalculatorMode(&fingerMovie, inOutControl->in, inOutControl->out, Mode);
		if(backgroundMethod == BACKGROUND_METHOD_MEDIAN)
			backgroundCalculator = new BackgroundCalculatorMode(&fingerMovie, inOutControl->in, inOutControl->out, Median);

		backgroundDisplay->content = & backgroundCalculator->currentBackground;

		backgroundIsStale = true;
		backgroundIsUpdating = false;

		lastBackgroundMethod = backgroundMethod;
		lastIn = inOutControl->in;
		lastOut = inOutControl->out;
	}
	




	//printf("%i\n", saveBackground);
}

//--------------------------------------------------------------
void BackgroundToolApp::draw(){

	ofSetHexColor(0xFFFFFF);




    //fingerMovie.draw(20,20);
    ofSetHexColor(0x000000);
    //unsigned char * pixels = fingerMovie.getPixels();

	if(backgroundIsStale && !inOutControl->isInteracting())
	{
		if(!backgroundIsUpdating /*&& !dragging*/)
		{

			
			

			backgroundCalculator->reset();

			backgroundIsUpdating = true;
		}
		
		if(!dragging)
		{
			//cout << "original frame: " << fingerMovie.getWidth() << " " << fingerMovie.getHeight();
			backgroundCalculator->step();
			backgroundPercent = backgroundCalculator->getProgress();
			//background = backgroundCalculator->currentBackground;
		}



		//if(lastFrame >= outFrame && !dragging)
		if ( backgroundCalculator->isFinished() )
		{
			//backgroundCalculator->finish();
			cout << "done" << endl;
			backgroundIsUpdating = false;
			backgroundIsStale = false;
		}
		//background.setFromPixels(fingerMovie.getPixelsRef());
	}


    ofSetHexColor(0x000000);
	ofDrawBitmapString("press f to change",20,320);
    if(frameByframe) ofSetHexColor(0xCCCCCC);
    ofDrawBitmapString("mouse speed position",20,340);
    if(!frameByframe) ofSetHexColor(0xCCCCCC); else ofSetHexColor(0x000000);
    //ofDrawBitmapString("keys <- -> frame by frame " ,190,340);
    ofSetHexColor(0x000000);

    ofDrawBitmapString("frame: " + ofToString(fingerMovie.getCurrentFrame()) + "/"+ofToString(fingerMovie.getTotalNumFrames()),20,380);
    ofDrawBitmapString("duration: " + ofToString(fingerMovie.getPosition()*fingerMovie.getDuration(),2) + "/"+ofToString(fingerMovie.getDuration(),2),20,400);
    ofDrawBitmapString("speed: " + ofToString(fingerMovie.getSpeed(),2),20,420);



	gui.draw();

}

//--------------------------------------------------------------
void BackgroundToolApp::keyPressed  (int key){
    switch(key){
        case 'f':
            frameByframe=!frameByframe;
            fingerMovie.setPaused(frameByframe);
        break;
        case OF_KEY_LEFT:
            fingerMovie.previousFrame();
        break;
        case OF_KEY_RIGHT:
            fingerMovie.nextFrame();
        break;
        case '0':
            fingerMovie.firstFrame();
        break;
    }
}

//--------------------------------------------------------------
void BackgroundToolApp::keyReleased(int key){

}

//--------------------------------------------------------------
void BackgroundToolApp::mouseMoved(int x, int y ){
	/*if(!frameByframe){
        int width = ofGetWidth();
        float pct = (float)x / (float)width;
        float speed = (2 * pct - 1) * 5.0f;
        fingerMovie.setSpeed(speed);
	}*/
}

//--------------------------------------------------------------
void BackgroundToolApp::mouseDragged(int x, int y, int button){
	/*if(!frameByframe){
        int width = ofGetWidth();
        float pct = (float)x / (float)width;
        fingerMovie.setPosition(pct);
	}*/
}

//--------------------------------------------------------------
void BackgroundToolApp::mousePressed(int x, int y, int button){
	/*if(!frameByframe){
        fingerMovie.setPaused(true);
	}*/

	dragging = true;
}


//--------------------------------------------------------------
void BackgroundToolApp::mouseReleased(int x, int y, int button){
	/*if(!frameByframe){
        fingerMovie.setPaused(false);
	}*/

	dragging = false;
}

//--------------------------------------------------------------
void BackgroundToolApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void BackgroundToolApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void BackgroundToolApp::dragEvent(ofDragInfo dragInfo){ 

}
