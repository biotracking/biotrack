#include "mugshot.h"

Mugshot::Mugshot(QString videopath, QString bgImagepath, QString savepath){


    bgImage = cvLoadImage(bgImagepath.toAscii());
    path= savepath;


    targetX = 100;
    targetY = 60;
    maxXY=0;
    croppedWidth = 120;
    croppedHeight = 120;
    temp = 0;
    currentFrame=1;
    maxXY=max(targetX,targetY);

    xcent=1920/2;
    ycent=1080/2;
    offset=200;
    antWidth=200;
    antHeight=310;


    totalspots=20;
    anglechange=0.31415; //18 degrees in radians



}

Mugshot::~Mugshot(){
    cvReleaseImage(&bgImage);
    delete bgImage;

//Uncomment when they get ported over
//    cvReleaseImage(&frameGray);
//    cvReleaseImage(&frameorig);
}

Mat Mugshot::rotateImage(const Mat& source, double anglerad)
{
    double angle  = ((anglerad*180)/CV_PI) ;
    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
}

void Mugshot::loop(IplImage* frame)
{

    frameorig=cvCloneImage(frame);

    qDebug()<<"size: "<<uCent.size()<<" centX: "<<uCent[0].x()<<" centY: "<<uCent[0].y();
    for(int i = 0; i < uCent.size(); i++) {
        double controlAngle=0;
        tempframe=cvCloneImage(frame);
        frameGray = cvCreateImage(cvGetSize(frame),frame->depth,1);

        cvAbsDiff(frame,bgImage, tempframe);

        cvCvtColor(tempframe,frameGray, CV_RGB2GRAY);// rgb -> gray

        cvThreshold(frameGray,frameGray,50,255,CV_THRESH_BINARY); //threshold the gray
        cvCopy(frame,tempframe,frameGray);//Apply mask

        //bg sub end


        Mat frameMat(tempframe);

        //Center ant image

        int diffX = abs(uMax[i].x()-uMin[i].x());
        int diffY =abs(uMax[i].y()-uMin[i].y());

        //Offset center
        x1=uCent[i].x()-diffX/2;
        y1=uCent[i].y()-diffY/2;
        x2=diffX;
        y2=diffY;

        //rotframe = rotateImage(frameMat, controlAngle);//Rotate full image about this center

        cv::Rect myROI(x1,y1,x2,y2);
        cropframe = frameMat(myROI);
        int nonblack= 0;
        Vec3b pixel;
        Point a;

        uchar blue;
        uchar green;
        uchar red;
        //qDebug()<<"Size: "<<cropframe.rows*cropframe.cols;

        Point coordinate;

        for (int y=0; y < cropframe.rows; y++)
        {
            for (int x=0; x < cropframe.cols; x++)
            {
               // pixel = cvGet2D(tempImg, y, x);
                int pixval =cropframe.at<uchar>(x,y);
                    pixel = cropframe.at<Vec3b>(x, y);

                    pixval = (pixel.val[0] + pixel.val[1] + pixel.val[2])/3;

                if (pixval > 40 && pixval<240)
                {

                   nonblack +=1;

                }
            }
        }

            qDebug()<<"Frame: "<<currentFrame<<" ROI: "<<i<<" nonblack: "<<nonblack;
            imgRot = cropframe;
            QString ext = ".png";
            QString filename = path+"framenum_"+QString::number(currentFrame)+"_Spotnum_"+QString::number(i)+ext;
            //cout << "Save out " << filename.toStdString() << endl ;
            if(nonblack>2000){
                cvSaveImage(filename.toAscii(), &imgRot,0);
            }
            currentFrame++;




        /* debuging */
        //qDebug() << "center of ant large" << ((&img)->width/2-40) << " " << ((&img)->height/2-20) ;

       if( !frame ) break;

       cvReleaseImage(&frameGray);
       cvReleaseImage(&frameorig);
       cvReleaseImage(&tempframe);
    }
}



