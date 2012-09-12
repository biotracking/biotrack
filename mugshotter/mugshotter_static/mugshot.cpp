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
    skip=1;
    maxXY=max(targetX,targetY);

    xcent=1920/2;
    ycent=1080/2;
    offset=200;
    antWidth=200;
    antHeight=310;


    totalspots=20;
    anglechange=0.31415; //18 degrees in radians
    retSubtract=false;
    threshold =15000;
    lineType = 8;


}

    Mugshot::~Mugshot(){
        cvReleaseImage(&bgImage);
        delete bgImage;
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
    if(frame!=NULL){
        frameorig=cvCloneImage(frame);
        Point img_border[1][4];
        img_border[1][0]=Point(0,0);
        img_border[1][0]=Point(frame->width,0);
        img_border[1][0]=Point(0,frame->height);
        img_border[1][0]=Point(frame->width,frame->height);

        const Point* border_ppt[1] = { img_border[0] };
        int border_npt[] = { 4 };


        //qDebug()<<"size: "<<uCent.size()<<" centX: "<<uCent[0].x()<<" centY: "<<uCent[0].y();
        for(int i = 0; i < uCent.size(); i++) {
            nonblack= 0;

            tempframe=cvCloneImage(frame);
            maskframe=cvCloneImage(frame);

            //std::cout<<"WTF!"<<std::endl;
            frameGray = cvCreateImage(cvGetSize(frame),frame->depth,1);
            cvAbsDiff(frame,bgImage, tempframe);
            cvCvtColor(tempframe,frameGray, CV_RGB2GRAY);// rgb -> gray
            cvThreshold(frameGray,frameGray,50,255,CV_THRESH_BINARY); //threshold the gray
            cvCopy(frame,tempframe,frameGray);//Apply mask


            Mat frameMat(tempframe);
            Mat mask = frameMat.clone();



            rectangle(mask, Point( 0, 0 ), Point( mask.cols, mask.rows), Scalar( 0, 0,0 ),-1, 8 );
            //Polgon Masking
            polygon = polygons.at(i);


            Point poly_points[polygon.size()];

            for(int j=0;j<polygon.size();j++)
            {
                poly_points[j]=Point(3*polygon.at(j).x(), 3*polygon.at(j).y());;
            }

            const Point* ppt[1] = { poly_points };
            int npt[] = { polygon.size() };

            fillPoly( mask,
                      ppt,
                      npt,
                      1,
                      Scalar( 255, 255,255 ),
                      lineType,
                      0);



            int diffX = abs(uMax[i].x()-uMin[i].x());
            int diffY = abs(uMax[i].y()-uMin[i].y());

            x1=uMin[i].x();
            y1=uMin[i].y();
            x2=diffX;
            y2=diffY;

    //        rotframe = rotateImage(frameMat, controlAngle);//Rotate full image about this center

            cv::Rect myROI(x1,y1,x2,y2);

            bitwise_and(frameMat, mask, frameMat);
            cropframe = frameMat(myROI);

            for (int j=0; j<cropframe.rows; j++) {
                // get the address of row j
                uchar* data= cropframe.ptr<uchar>(j);
                for (int i=0; i<cropframe.cols * cropframe.channels(); i++) { // process each pixel ---------------------
                    if (data[i] > 40 && data[i]<240)
                    {
                       nonblack +=1;
                    }
                }
            }


//            qDebug()<<"Frame: "<<currentFrame<<" ROI: "<<i<<" nonblack: "<<nonblack<<" threshold: "<<threshold;
            imgRot = cropframe;


            QString ext = ".png";

            if(!QFile::exists(path+polyNames.at(i))){
                qDebug()<<"DNE!";
                sub.mkdir(path+polyNames.at(i));
            }

            QString filename = path+polyNames.at(i)+"/"+polyNames.at(i)+"_"+QString::number(uCent[i].x())+"_"+QString::number(uCent[i].y())+"_"+QString::number(currentFrame)+ext;


            if(nonblack>threshold){
                cvSaveImage(filename.toAscii(), &imgRot,0);
                qDebug()<<"Saved out: "<<polyNames.at(i)<<" to "<<filename      ;
            }

           if( !frame ) break;

           cvReleaseImage(&frameGray);
           cvReleaseImage(&frameorig);
           cvReleaseImage(&tempframe);
           cvReleaseImage(&maskframe);

        }
        currentFrame = currentFrame+skip;

    }
}

IplImage * Mugshot::subtractBack(IplImage *img)
{
       IplImage* temp=cvCloneImage(img);
        IplImage* tempGray = cvCreateImage(cvGetSize(img),img->depth,1);
        cvAbsDiff(img,bgImage, temp);
        cvCvtColor(temp,tempGray, CV_RGB2GRAY);// rgb -> gray
        cvThreshold(tempGray,tempGray,50,255,CV_THRESH_BINARY); //threshold the gray
        cvCopy(img,temp,tempGray);//Apply mask

        return temp;

}

Mat maskPoly(Mat img , vector<QPoint> points)
{

    int lineType = 8;

    Point poly_points[1][points.size()];

    for(int i=0;i<points.size();i++)
    {
        poly_points[1][i]=Point(points.at(i).x(), points.at(i).y());;
    }

    const Point* ppt[1] = { poly_points[0] };
    int npt[] = { points.size() };

    fillPoly( img,
              ppt,
              npt,
              1,
              Scalar( 255, 255, 255 ),
              lineType );

}

void Mugshot::writePoly()
{
    QFile file(path+"polygons.txt");
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    QString polyout;
    int current=0;
    for (row = polygons.begin(); row != polygons.end(); ++row) {
        polyout+=polyNames.at(current)+",";
        for (col = row->begin(); col != row->end(); ++col) {
            tempV =*row;
            tempPoint = *col;

            if(tempPoint==tempV.back()){
              polyout += "("+QString::number(tempPoint.x())+","+QString::number(tempPoint.y())+")";

            } else {
              polyout += "("+QString::number(tempPoint.x())+","+QString::number(tempPoint.y())+"),";
            }
        }
        out<< polyout+"\n";
        polyout="";
        current++;
        qDebug()<<tempV.data();
    }
    // optional, as QFile destructor will already do it:
    file.close();
}
