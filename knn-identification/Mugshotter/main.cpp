#include <QtGui/QApplication>
#include "mugshotter.h"
#include <QString>
#include <QDir>
using namespace cv;

QString prefix("/media/8865399a-a349-43cd-8cc0-2b719505efaf/");
//bool showSpots = false;
int xOffset, yOffset, xcent, ycent;
int radialOffset=170;
int antWidth=200;
int antHeight=260;
int totalspots=40;
double anglechange=3.14159/20; //9 degrees in radians

Mat rotateImage(const Mat& source, double anglerad)
{
    double angle  = ((anglerad*180)/CV_PI) ;
   // qDebug() << "D " << angle ;
    //angle = 90 - angle;
    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;

}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    if(argc == 2){
        xOffset = 0;
        yOffset = 0;
        radialOffset=170;
        xcent=1920/2+xOffset;
        ycent=1080/2+yOffset;
    }
    else if(argc == 5){
        xOffset = atoi(argv[2]);
        yOffset = atoi(argv[3]);
        radialOffset = atoi(argv[4]);
        xcent=1920/2+xOffset;
        ycent=1080/2+yOffset;
    }
    else{
        std::cout<<"Invalid Arguments. Usage: \"./Mugshotter Folder_Name/\" or \n\"./Mugshotter Folder_Name/ X_Offset Y_Offset Radial_Offset\""<<std::endl;
        return -1;
    }
    prefix += argv[1];
    std::cout<<"Using arguments xOffset = "<<xOffset<<", yOffset = "<<yOffset<<", radialOffset = "<<radialOffset<<std::endl;

    QDir dir(prefix);
    QStringList fileList = dir.entryList(QStringList("*.MOV"));
    QStringList::const_iterator uIterator;
    for (uIterator = fileList.constBegin(); uIterator != fileList.constEnd(); ++uIterator){
		CvCapture* capture = NULL;
        QString name = (*uIterator);
        QString fileName(prefix+name);
        QString path = prefix+"FeederMugshots_"+name.remove(QString(".MOV"))+"/"; //save directory
        QString ext = ".png";
		std::cout<<"Opening "<<fileName.toStdString()<<std::endl;
		capture = cvCreateFileCapture(fileName.toAscii());
		if(!capture) std::cout<<"Bad Capture"<<std::endl;

		IplImage* frame;
        cv::Mat cropframe;
		cv::Mat rotframe;
        IplImage* frameOrig;
		IplImage* bgImage = cvLoadImage(QString(prefix+"background.png").toAscii());
		IplImage* frameGray;
		IplImage* frameSize;

		int x1,y1;
		int currentFrame=1;

		cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES,currentFrame);
		do{
			if(currentFrame%10==0)
            std::cout<<(*uIterator).toStdString()<<":"<<currentFrame<<std::endl;

			//background subtraction
			frame = cvQueryFrame(capture);
			if( !frame ) break;
            frameOrig = cvCloneImage(frame);
			frameGray = cvCreateImage(cvGetSize(frame),frame->depth,1);

			frameSize = cvCloneImage(frame);
			cvResize(bgImage,frameSize);
			cvAbsDiff(frame,frameSize,frame);
			cvCvtColor(frame,frameGray, CV_RGB2GRAY);// rgb -> gray
			cvThreshold(frameGray,frameGray,50,255,CV_THRESH_BINARY); //threshold the gray

            cvCopy(frameOrig,frame,frameGray);//Apply mask

			for(int currentSpot=0; currentSpot<totalspots;currentSpot++){
				double controlAngle=0+anglechange*currentSpot;
				Mat frameMat(frame);

				//Center ant image
				x1=xcent;
				y1=ycent;

				rotframe = rotateImage(frameMat, controlAngle);//Rotate full image about this center
                cropframe = rotframe(cv::Rect(x1,y1+radialOffset,antWidth,antHeight));//Adjust the center

				IplImage imgRot;
				imgRot = cropframe;

				//count non-black pixels in imgRot to see if it is worth saving
				RgbImage img(&imgRot);
				int numNonBlack = 0;
				for(int r=0; r<imgRot.height; r++){
					for(int c=0; c<imgRot.width; c++){
						if(img[r][c].r>50 || img[r][c].g>50 || img[r][c].b>50)
							numNonBlack++;
					}
				}

				if(numNonBlack>10000){ //this number is pretty arbitrary
					//std::cout<<currentFrame<<","<<currentSpot<<"="<<numNonBlack<<std::endl;

                    QString filename = path+"/pSpot"+QString::number(currentSpot)+"/framenum_"+QString::number(currentFrame)+ext;
                    QDir dir(path+"/pSpot"+QString::number(currentSpot)+"/"); if (!dir.exists()) dir.mkpath(".");
					cvSaveImage(filename.toAscii(), &imgRot,0);

				}

                cvReleaseImage(&frameGray);
                cvReleaseImage(&frameOrig);
				cvReleaseImage(&frameSize);
			}

            if(currentFrame == 60){ //give some time for the camera to settle before taking the debug pic
                for(int currentSpot=0; currentSpot<totalspots;currentSpot++){
                    double controlAngle=0+anglechange*currentSpot;
                    CvPoint c1 = cvPoint(xcent-(radialOffset*sin(controlAngle)),ycent+(radialOffset*cos(controlAngle)));
                    CvPoint r1 = cvPoint(c1.x-sin(controlAngle+1.57079633)*(antWidth/2),c1.y+cos(controlAngle+1.57079633)*(antWidth/2));
                    CvPoint r2 = cvPoint(c1.x+sin(controlAngle+1.57079633)*(antWidth/2),c1.y-cos(controlAngle+1.57079633)*(antWidth/2));
                    CvPoint c2 = cvPoint(xcent-((radialOffset+antHeight)*sin(controlAngle)),ycent+((radialOffset+antHeight)*cos(controlAngle)));
                    CvPoint r3 = cvPoint(c2.x+sin(controlAngle+1.57079633)*(antWidth/2),c2.y-cos(controlAngle+1.57079633)*(antWidth/2));
                    CvPoint r4 = cvPoint(c2.x-sin(controlAngle+1.57079633)*(antWidth/2),c2.y+cos(controlAngle+1.57079633)*(antWidth/2));

                    cvLine(frame, r1, r2, cvScalar(0,255,255,0), 3);
                    cvLine(frame, r2, r3, cvScalar(0,255,255,0), 3);
                    cvLine(frame, r3, r4, cvScalar(0,255,255,0), 3);
                    cvLine(frame, r4, r1, cvScalar(0,255,255,0), 3);

                    //char IDbuff[3];
                    //sprintf(IDbuff,"%d",currentSpot);
                    //cvPutText(frame, IDbuff, cvPoint(.5*(c1.x+c2.x)-50,.5*(c1.y+c2.y)));
                }

                QString filename = path + "debug/" + QString::number(currentFrame) + ext;
                QDir dir(path + "debug/"); if (!dir.exists()) dir.mkpath(".");
                cvSaveImage(filename.toAscii(), frame, 0);
            }

            currentFrame++;
			//cvReleaseImage(&frame);

		}while(cvGetCaptureProperty(capture,CV_CAP_PROP_POS_AVI_RATIO)<.99);

		cvReleaseImage(&bgImage);
		cvReleaseCapture( &capture );
        std::cout<<"Done with clip "<<name.toStdString()<<std::endl;
    }
    std::cout<<"Done"<<std::endl;
    return a.exec();
}




