#include <QtCore>
#include <QCoreApplication>
#include "/home/stephen/OpenCV-2.3.1/include/opencv/cv.h"
#include "/home/stephen/OpenCV-2.3.1/include/opencv/highgui.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

QString prefix("../data/");
bool aviOut = false;
bool showDebug = false;
int numParkingSpots = 40;
int numFrames = 22000;

CvVideoWriter *writer;
CvFont font;
CvFont bigFont;
//CvScalar trueColor = cvScalar(0, 255, 0, 0);
//CvScalar otherColor = cvScalar(0, 255, 255, 0);
int **spotIDs; //[frame, spot] => ID

//QString IDbuff[ ] = {
//        QString("-ER-"),
//        QString("BGBY"),
//        QString("GBGP"),
//        QString("GPBW"),
//        QString("BPPO"),
//        QString("OGPG"),
//        QString("PGOP"),
//        QString("BPWP"),
//        QString("GWBG"),
//        QString("GWOW"),
//        QString("OP--"),
//        QString("PP--"),
//        QString("GOYP"),
//        QString("OOOW"),
//        QString("OPYO"),
//        QString("GW--"),
//        QString("GWPB"),
//        QString("BBGY"),
//        QString("GO-B")};
QString IDbuff[ ] = {
        QString("-ER-"),
        QString("----"),
        QString("--BP"),
        QString("--G-"),
        QString("-B--"),
        QString("-BYR"),
        QString("-RPG"),
        QString("-W--"),
        QString("-WGR"),
        QString("-WRG"),
        QString("-Y--"),
        QString("-YRW"),
        QString("BBGY"),
        QString("BGBY"),
        QString("BGY-"),
        QString("BPPG"),
        QString("BPPO"),
        QString("BPWP"),
        QString("BPYB"),
        QString("BRWR"),
        QString("BY--"),
        QString("BYBG"),
        QString("BYPB"),
        QString("BYYP"),
        QString("GB--"),
        QString("GB-O"),
        QString("GBBY"),
        QString("GBGP"),
        QString("GBRB"),
        QString("GG-O"),
        QString("GGGB"),
        QString("GO-B"),
        QString("GOYP"),
        QString("GPBW"),
        QString("GPGY"),
        QString("GPPR"),
        QString("GRBW"),
        QString("GRGP"),
        QString("GW--"),
        QString("GWBG"),
        QString("GWOW"),
        QString("GWPB"),
        QString("GY--"),
        QString("GYPG"),
        QString("OGPG"),
        QString("OOO-"),
        QString("OOOW"),
        QString("OP--"),
        QString("OPYO"),
        QString("OWGW"),
        QString("P---"),
        QString("PGOP"),
        QString("PP--"),
        QString("PYWB"),
        QString("R-BP"),
        QString("RBGB"),
        QString("RBRG"),
        QString("RGGP"),
        QString("RRBG"),
        QString("RWYB"),
        QString("RYRP"),
        QString("RYYG"),
        QString("WPWO")};

CvScalar colors[ ] = {
        cvScalar(0, 0, 0, 0),
        cvScalar(255, 0, 0, 0),
        cvScalar(0, 255, 0, 0),
        cvScalar(0, 0, 255, 0),
        cvScalar(255, 255, 0, 0),
        cvScalar(0, 255, 255, 0),
        cvScalar(255, 0, 255, 0),
        cvScalar(255, 255, 255, 0),
        cvScalar(127, 20, 20, 0),
        cvScalar(20, 127, 20, 0),
        cvScalar(20, 20, 127, 0),
        cvScalar(127, 127, 20, 0),
        cvScalar(20, 127, 127, 0),
        cvScalar(127, 20, 127, 0)};

void processClip(int clipNum){
	
    int **spotIDs = new int*[numFrames];
    int **blurBuffer = new int*[numFrames];
    for(int i=0 ; i<numFrames ; i++){
    	spotIDs[i] = new int[numParkingSpots];
    	blurBuffer[i] = new int[numParkingSpots];
    }
    double* certainties = new double[numParkingSpots*numFrames];

    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);
	cvInitFont(&bigFont, CV_FONT_HERSHEY_SIMPLEX, 1.1, 1.1, 0, 2.5, CV_AA);

	if(aviOut){
		writer = 0;
		int fps = 30;
		int frameW = 1920;//1920;
		int frameH = 1080;//1080;
		writer=cvCreateVideoWriter(QString(prefix+"experiment.avi").toAscii(),CV_FOURCC('D','I','V','X'),//CV_FOURCC_DEFAULT,//
									   fps,cvSize(frameW,frameH),1);
		if (writer==0)
			qDebug()<<"could not open writer";
	}
	if(showDebug){
        cvNamedWindow("Debug", CV_WINDOW_AUTOSIZE );
	}
    CvCapture* capture = cvCreateFileCapture(QString(prefix+"B144_3.21.12_0.1M_clip"+QString::number(clipNum)+".MOV").toAscii());
	IplImage* frame;
	int currentFrame=1;
	cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, currentFrame);

    QString name = prefix+"results"+QString::number(clipNum)+".csv";
    ifstream myfile (name.toUtf8().constData());
	if(!myfile.is_open()) exit(0);
	string line;
	if(myfile.good()) getline (myfile,line); //the header line
	//make array of spots in frames
	for(int i=0; i<numFrames; i++){
		for(int j=0; j<numParkingSpots; j++){
			spotIDs[i][j] = -1;
			blurBuffer[i][j] = -1;
			certainties[i*numParkingSpots+j] = -1;
		}
	}
    while(myfile.good()){
    	int frameNum, spotNum, id;
    	double trash, similarity;
    	char commaTrash;
		myfile >> frameNum;
		myfile >> commaTrash;
		myfile >> spotNum;
		myfile >> commaTrash;
		myfile >> id;
		myfile >> commaTrash;
		myfile >> trash;
		myfile >> commaTrash;
		myfile >> similarity;
		myfile >> commaTrash;
        if(similarity > .8){ //hack?
            blurBuffer[frameNum][spotNum] = id;
            certainties[(frameNum)*numParkingSpots+spotNum] = similarity;
        }
	}

	//filter
//	//Fix small flickers by temporal blurring
//	for(int j=0; j<numParkingSpots; j++){
//		for(int i=3; i<numFrames-3; i++){
//			int votesForID[20];
//			for(int k=0; k<20; k++) votesForID[k]=0;
//			for(int k=i-3; k<i+3; k++) votesForID[blurBuffer[k][j]]++; //blur size 7
//			for(int k=0; k<20; k++) if(votesForID[k]>=4) spotIDs[i][j] = k;
//		}
//	}

	//alternative (better) filtering : maintain id from when spot is occupied to when it is vacant
	//(((filter per space)))
	for(int j=0; j<numParkingSpots; j++){
		for(int start=0; start<numFrames; start++){
			if(blurBuffer[start][j] >= 0){
				int end = start;
				while(end < numFrames-1 && blurBuffer[end][j] >= 0){
					end++;
				}
				//erase really quick visits (100 frames or less) ***OR ONES WITH LOW CERTAINTY?***
				if(end-start < 100){
					for(int i=start; i<end; i++){
						spotIDs[i][j] = -1;
						blurBuffer[i][j] = -1;
					}
				}
				else{
                    int max = 0, votesForID[80];
                    for(int k=0; k<80; k++) votesForID[k]=0;
					for(int i=start; i<end; i++) votesForID[blurBuffer[i][j]]++;
                    for(int k=0; k<80; k++)	if(votesForID[k] > votesForID[max]) max = k;
					for(int i=start; i<end; i++){
						spotIDs[i][j] = max;
						blurBuffer[i][j] = -1;
					}
				}
			}
		}
	}
	//(((filter between spaces)))
	for(int i=0; i<numFrames; i++){
		for(int j=0; j<numParkingSpots; j++){
			if(spotIDs[i][j] >= 0){
				int previousJ = j-1;
				if(previousJ < 0) previousJ = numParkingSpots-1;
				if(spotIDs[i][previousJ] >= 0 && spotIDs[i][j] != spotIDs[i][previousJ]){
					//two, non-empty spaces with different IDs:
					//back-project onto original image (can't do this without knowing more about the mugshots) ~ more detail in the file
					//what we can do is remove the solo images (i.e. no real ant is just in one space, there is always some overlap)
					int nextJ = j+1;
					if(nextJ >= numParkingSpots) nextJ = 0;
					int twoPreviousJ = previousJ - 1;
					if(twoPreviousJ < 0) twoPreviousJ = numParkingSpots-1;
					if(spotIDs[i][j] == spotIDs[i][nextJ] && spotIDs[i][twoPreviousJ] != spotIDs[i][previousJ]){
						//if we agree with our other neighbor and the other guy does not, bring him to the majority side
						spotIDs[i][previousJ] = spotIDs[i][j];
					}
					else if(spotIDs[i][j] != spotIDs[i][nextJ] && spotIDs[i][twoPreviousJ] == spotIDs[i][previousJ]){
						//if we don't agree with our other neighbor and the other guy does, bring us to the majority side
						spotIDs[i][j] = spotIDs[i][previousJ];
					}
				}
			}
		}
	}
	//some more short visits may have popped up, go back and get rid of them
	for(int j=0; j<numParkingSpots; j++){
		for(int start=0; start<numFrames; start++){
			if(spotIDs[start][j] >= 0){
				int end = start;
				while(end < numFrames-1 && spotIDs[end][j] == spotIDs[start][j]){
					end++;
				}
				//erase really quick visits (100 frames or less)
				if(end-start < 100){
					for(int i=start; i<end; i++){
						spotIDs[i][j] = -1;
					}
				}
				else{
					start = end-1;
				}
			}
		}
	}
	ofstream outFile;
    name = prefix+"filtered"+QString::number(clipNum)+".csv";
	outFile.open(name.toUtf8().constData());
    outFile << "Frame Number, Spot Number, ID, Confidence (unused), Similarity (unused), \n";
	for(int i=0; i<numFrames; i++){
		for(int j=0; j<numParkingSpots; j++){
			if(spotIDs[i][j] >= 0)
                outFile << i << "," << j << "," << IDbuff[spotIDs[i][j]].toStdString() << "," << 0 << "," << 0 << ", \n"; //better values than 0 would be nice
		}
	}
	outFile.close();
        qDebug()<<"Processing complete for clip"<<clipNum;

        if(!(showDebug || aviOut))
            return;

	//Assign 'true position' to average of observed positions
	for(int i=0; i<numFrames; i++){
		for(int j=0; j<numParkingSpots; j++){
			if(spotIDs[i][j]>=0){
				//find center
				int max = j, min = j;
				while(spotIDs[i][max] == spotIDs[i][j]){
					max++;
					if(max>=numParkingSpots)
						max = 0;
				}
				while(spotIDs[i][min] == spotIDs[i][j]){
					min--;
					if(min<0)
						min = numParkingSpots-1;
				}
				int avg = (max>min) ? (max+min)/2 : (40+max+min)/2;
				if(avg >= numParkingSpots) avg -= numParkingSpots;
				int id = spotIDs[i][j];
				for(int k=0; k<numParkingSpots; k++){
					if (spotIDs[i][k] == id)
						spotIDs[i][k] = -1;
					if (k==avg)
						spotIDs[i][k] = id;
				}
			}
		}
	}

    //draw spots on frames
	do {
		if(currentFrame%50==0)
			qDebug()<<"Rendering frame"<<currentFrame;

	    int xcent = 1970/2;
	    int ycent = 1080/2;
	    int offset = 180;//200;
	    int antWidth = 200;
	    int antHeight = 260;
	    int totalspots = numParkingSpots;
	    double anglechange = 3.14159/20; //9 degrees in radians
	    frame = cvQueryFrame(capture);
	    if( !frame ) break;
	    IplImage* originalFrame = cvCloneImage( frame );

	    //Draw spots
	    for(int currentSpot=0; currentSpot<totalspots; currentSpot++){
	    	if(spotIDs[currentFrame][currentSpot] >= 0 && spotIDs[currentFrame][currentSpot]<20){
	    		//qDebug()<<currentFrame<<currentSpot<<spotIDs[currentFrame][currentSpot];
	    		//CvScalar spotColor = (spotIDs[currentFrame][currentSpot]<20) ? trueColor : otherColor;

                                double controlAngle = 0 + anglechange * (currentSpot-11); //please fix me
				CvPoint c1 = cvPoint(xcent-(offset*sin(controlAngle)),ycent+(offset*cos(controlAngle)));
				CvPoint r1 = cvPoint(c1.x-sin(controlAngle+1.57079633)*(antWidth/2),c1.y+cos(controlAngle+1.57079633)*(antWidth/2));
				CvPoint r2 = cvPoint(c1.x+sin(controlAngle+1.57079633)*(antWidth/2),c1.y-cos(controlAngle+1.57079633)*(antWidth/2));
				CvPoint c2 = cvPoint(xcent-((offset+antHeight)*sin(controlAngle)),ycent+((offset+antHeight)*cos(controlAngle)));
				CvPoint r3 = cvPoint(c2.x+sin(controlAngle+1.57079633)*(antWidth/2),c2.y-cos(controlAngle+1.57079633)*(antWidth/2));
				CvPoint r4 = cvPoint(c2.x-sin(controlAngle+1.57079633)*(antWidth/2),c2.y+cos(controlAngle+1.57079633)*(antWidth/2));

				cvLine(frame, r1, r2, colors[(spotIDs[currentFrame][currentSpot]<20) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 20], 3);
				cvLine(frame, r2, r3, colors[(spotIDs[currentFrame][currentSpot]<20) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 20], 3);
				cvLine(frame, r3, r4, colors[(spotIDs[currentFrame][currentSpot]<20) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 20], 3);
				cvLine(frame, r4, r1, colors[(spotIDs[currentFrame][currentSpot]<20) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 20], 3);

				//char IDbuff[10];
				//sprintf(IDbuff,"%d",(spotIDs[currentFrame][currentSpot]<10) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 10);
				//qDebug()<<spotIDs[currentFrame][currentSpot];
				cvPutText(frame, IDbuff[(spotIDs[currentFrame][currentSpot]<20) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 20].toAscii(),
						cvPoint(.5*(c1.x+c2.x)-50,.5*(c1.y+c2.y)),
						&bigFont, colors[(spotIDs[currentFrame][currentSpot]<20) ? spotIDs[currentFrame][currentSpot] : spotIDs[currentFrame][currentSpot] - 20]);
	    	}
	    }
	    int frameW = 1920, frameH = 1080;
	    for(int timeFrames = 0; timeFrames < numFrames; timeFrames++){
			for(int currentSpot=0; currentSpot<totalspots; currentSpot++){
				if(spotIDs[timeFrames][currentSpot] >= 0){
					CvPoint p1 = cvPoint((int)(frameW*((double)timeFrames/numFrames)),frameH+spotIDs[timeFrames][currentSpot]*5-100);
					CvPoint p2 = cvPoint((int)(frameW*((double)(timeFrames-1)/numFrames)),frameH+spotIDs[timeFrames][currentSpot]*5-100);
					cvLine(frame, p1, p2, colors[(spotIDs[timeFrames][currentSpot]<20) ? spotIDs[timeFrames][currentSpot] : spotIDs[timeFrames][currentSpot] - 20], 3);
					cvLine(originalFrame, p1, p2, colors[(spotIDs[timeFrames][currentSpot]<20) ? spotIDs[timeFrames][currentSpot] : spotIDs[timeFrames][currentSpot] - 20], 3);
					//cvLine(originalFrame, cvPoint(0,frameH+spotIDs[timeFrames][currentSpot]*5-100), cvPoint(frameW,frameH+spotIDs[timeFrames][currentSpot]*5-100), colors[(spotIDs[timeFrames][currentSpot]<20) ? spotIDs[timeFrames][currentSpot] : spotIDs[timeFrames][currentSpot] - 20], 3);
				}
			}
	    }
	    CvPoint p1 = cvPoint((int)(frameW*((double)currentFrame/numFrames)),frameH-100);
		CvPoint p2 = cvPoint((int)(frameW*((double)(currentFrame)/numFrames)),frameH);;
		cvLine(frame, p1, p2, colors[0], 2);
		cvLine(originalFrame, p1, p2, colors[0], 2);

	    char IDbuff[10];
		sprintf(IDbuff,"Frame %d",currentFrame);
		cvPutText(frame, IDbuff,
				cvPoint(10,40),
				&bigFont, colors[0]);
		cvPutText(originalFrame, IDbuff,
				cvPoint(10,40),
				&bigFont, colors[0]);

		cvAddWeighted(frame, 0.5, originalFrame, 0.5, 0.0, frame);

		if(showDebug){
            cvShowImage("Debug", frame );
		}
		if(aviOut){
			if (!cvWriteFrame(writer,frame))
				qDebug()<<"could not write frame";
		}
	    currentFrame++;

		cvReleaseImage(&originalFrame);
                //cvReleaseImage(&frame);

            char c = cvWaitKey(5);
	    if( c == 'x' ) break;
	} while(cvGetCaptureProperty(capture,CV_CAP_PROP_POS_AVI_RATIO)<.99 && (showDebug || aviOut));
	if(showDebug){
                cvDestroyWindow("Debug");
	}
	if(aviOut){
		cvReleaseVideoWriter(&writer);
	}
	delete [] spotIDs;
	delete [] certainties;
	delete [] blurBuffer;
	qDebug()<<"Done";
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    if(argc == 2)
        prefix+=argv[1];
    else
        prefix+="B144/Feeder_1.6M/";

    for(int clip=1; clip<=8; clip++)
        processClip(clip);

    return a.exec();
}
