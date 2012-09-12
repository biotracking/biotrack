#include "background.h"
#include <iostream>
#include "time.h"
#include <fstream>
#include <string>
//#include <sstream>

#include "cv.h"
#include "highgui.h"
#include "cxcore.h"

using namespace std;

Background::Background(AbosPool *input, AbosPool *output,  char *outFolder, double alpha, int fps)
{
  stopped = false;
  readpool = input;
  writepool = output;

  //writer = video_output;
  this->outFolder = outFolder;

  this->fps = fps;
  this->alpha = alpha;
  bgSet = false;


  ////////////////////////////////////////////
  image = 0;
  hsv = 0;
  hue = 0;
  mask = 0;
  backproject = 0;
  histimg = 0;
  hist = 0;

  backproject_mode = 0;
  select_object = 0;
  track_object = 0;
  show_hist = 1;

  hdims = 16;
  hranges_arr[0]=0;
  hranges_arr[1]=180;
  hranges = hranges_arr;
  vmin = 100;
  vmax = 256;
  smin = 40;
}


void Background:: loadTemplateImage()
{
  string str_inTemplate(outFolder); 
  str_inTemplate += "/template.jpg";
  //str_inTemplate += "/tennis_ball.jpg";

  IplImage *src_tempimage = cvLoadImage(str_inTemplate.c_str(),1);
  if(src_tempimage == 0 )
    {
      std::cout<<"cannot load the template image!"<<std::endl;
      exit(0);
    }
  CvSize dst_cvsize;	
  dst_cvsize.width = hsv->width;		//目标图像的宽;the width of the target image
  dst_cvsize.height = hsv->height;	//目标图像的高;the height of the target image
 
  IplImage *dst_tempimage = cvCreateImage( dst_cvsize, src_tempimage->depth, src_tempimage->nChannels);	//构造目标图象;construct the target
  cvResize(src_tempimage, dst_tempimage, CV_INTER_LINEAR);	//缩放源图像到目标图像;the algorithm of zoom in/out
  cvCvtColor( dst_tempimage, hsv, CV_BGR2HSV );
  int _vmin = vmin, _vmax = vmax;
  // cvShowImage( "111", hsv );
  // cvWaitKey(10);
  cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
  	      cvScalar(180,255,MAX(_vmin,_vmax),0), mask );
		
  cvSplit( hsv, hue, 0, 0, 0 );
	
  selection.x = 1;
  selection.y = 1;
  // 	selection.width = 320-1;
  // 	selection.height= 240-1;
  selection.width = hsv->width-1;//480-1;
  selection.height= hsv->height-1;//360-1;

  cvSetImageROI( hue, selection );
  cvSetImageROI( mask, selection );
  cvCalcHist( &hue, hist, 0, mask );

  float max_val = 0.f;	
  
				
  cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
  cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
  cvResetImageROI( hue );
  cvResetImageROI( mask );
  track_window = selection;
  track_object = 1;
 std::cout<<"track_window in loadTemplate function : "<<track_window.x<<"  "<<track_window.y<<"  "<<track_window.width<<"  "<<track_window.height<<std::endl;

  cvZero( histimg );
  int bin_w = histimg->width / hdims;
  for(int i = 0; i < hdims; i++ )
    {
      int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
      CvScalar color = hsv2rgb(i*180.f/hdims);
      cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
  		   cvPoint((i+1)*bin_w,histimg->height - val),
  		   color, -1, 8, 0 );
    }

  cvReleaseImage(&src_tempimage);
  cvReleaseImage(&dst_tempimage);
}




CvScalar Background::hsv2rgb( float hue )
{
  int rgb[3], p, sector;
  static const int sector_data[][3]=
    {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
  hue *= 0.033333333333333333333333333333333f;
  sector = cvFloor(hue);
  p = cvRound(255*(hue - sector));
  p ^= sector & 1 ? 255 : 0;

  rgb[sector_data[sector][0]] = 255;
  rgb[sector_data[sector][1]] = 0;
  rgb[sector_data[sector][2]] = p;

  return cvScalar(rgb[2], rgb[1], rgb[0],0);
}



/**
   main loop of the thread
   take an image from the readpool,
   blur it and store to writepool.
   Other threads can be read the result of the blurred image
   from the writepool.
*/
void Background::run()
{
  ///////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////    
  // cvNamedWindow( "Histogram", 0 );
  // cvNamedWindow( "CamShiftDemo", 0 );
  //cvNamedWindow("111", 0);
  // cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
  //cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
  //cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
  //cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );
  //////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////
  int out_recordframeNO = 300;
  int in_videoframeNO = 300;
  int frameW  = 640; //  for firewire cameras
  int frameH  = 480; //  for firewire cameras
  int isColor = 1;
  string str_inTimeFile(outFolder);
  str_inTimeFile+="/ts.txt";
  ifstream inTimeFile(str_inTimeFile.c_str(), ios_base::in);

  string str_outVideo_circle(outFolder);
  str_outVideo_circle+="/out_circle.avi";
  CvVideoWriter *writer_circle = cvCreateVideoWriter(str_outVideo_circle.c_str(),CV_FOURCC('P','I','M','1'),
					      fps,cvSize(frameW,frameH),isColor);
  string str_outTimeFile_circle(outFolder);
  str_outTimeFile_circle+="/ts_circle.txt";
  ofstream outTimeFile_circle(str_outTimeFile_circle.c_str(), ios_base::out);

  int count_circle=0;
  int PreFrameNumber = 0;
 
  while(!stopped ){
    //std::cout<<"readpool->getSize() = "<<readpool->getSize()<<std::endl;
    PoolFrame *srcFrame;
    const cv::Mat* src_ID;
    // load an image from readpool
    if(readpool->getSize() == 0){
      srcFrame = NULL;
    }
    else{
      srcFrame = readpool->getRecentFrame();
      src_ID = srcFrame->getImage();
    }
    // processes background
    if(srcFrame != NULL){
      if( !bgSet ){
	prevBG = src_ID->clone();
	bgSet = false;
      }
      //////////////////////////////////////////////////////////////
      //Video frame processing	
      /////////////////////////////////////////////////////////////
      //addWeighted(*src_ID,alpha,prevBG,1-alpha,0.0,prevBG);
	   
      IplImage temp_frame=prevBG;
      IplImage* frame = cvCreateImage(cvGetSize(&temp_frame),
				      temp_frame.depth,
				      temp_frame.nChannels);
      cvCopy(&temp_frame,frame);
    

	
     // /////****
      if( !frame )
      	{
      	  std::cout<<"Could not initialize capturing!"<<std::endl;
      	  exit(0);
      	}
      int i, bin_w, c;

      //frame = cvQueryFrame( capture );
      std::cout<<"camera_width = "<<frame->width<<std::endl;
      std::cout<<"camera_height = "<<frame->height<<std::endl;
      if( !frame )
      	break;

      if( !image )
      	{
      	  /* allocate all the buffers */
      	  image = cvCreateImage( cvGetSize(frame), 8, 3 );
      	  image->origin = frame->origin;
      	  hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
      	  hue = cvCreateImage( cvGetSize(frame), 8, 1 );
      	  mask = cvCreateImage( cvGetSize(frame), 8, 1 );
      	  backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
      	  hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
      	  histimg = cvCreateImage( cvSize(320,200), 8, 3 );
      	  cvZero( histimg );

      	  loadTemplateImage();
      	}

      cvCopy( frame, image, 0);
      cvCvtColor( image, hsv, CV_BGR2HSV );
      std::cout<<"track_window : "<<track_window.x<<"  "<<track_window.y<<"  "<<track_window.width<<"  "<<track_window.height<<std::endl;
      if( track_object )
      	{
      	  int _vmin = vmin, _vmax = vmax;

      	  cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
      		      cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
      	  cvSplit( hsv, hue, 0, 0, 0 );

      	  cvCalcBackProject( &hue, backproject, hist );
      	  cvAnd( backproject, mask, backproject, 0 );
      	  cvCamShift( backproject, track_window,
      		      cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
      		      &track_comp, &track_box );
      	  if(track_comp.rect.width*track_comp.rect.height > 100&&track_comp.rect.width*track_comp.rect.height<480*360)
      	    track_window = track_comp.rect;
      	  else
      	    {
      	      std::cout<<"didnot match the template ROI target, so loop to search..."<<std::endl;
      	      //track_window.x = 1;
      	      //track_window.y = 1;
      	      //track_window.width = 640-1;
      	      //track_window.width = 480-1;
      	    }
		    
      	  if( backproject_mode )
      	    cvCvtColor( backproject, image, CV_GRAY2BGR );
      	  if( image->origin )
      	    track_box.angle = -track_box.angle;
			
      	  cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
      	}
		
      if( select_object && selection.width > 0 && selection.height > 0 )
      	{
      	  cvSetImageROI( image, selection );
      	  cvXorS( image, cvScalarAll(255), image, 0 );
      	  cvResetImageROI( image );
      	}

      //cvShowImage( "CamShiftDemo", image );
      // cvShowImage( "Histogram", histimg );
    
      c = cvWaitKey(10);
      if( c == 27 )
      	break;
      switch( c )
      	{
      	case 'b':
      	  backproject_mode ^= 1;
      	  break;
      	case 'c':
      	  track_object = 0;
      	  cvZero( histimg );
      	  break;
      	case 'h':
      	  show_hist ^= 1;
      	  if( !show_hist )
      	    cvDestroyWindow( "Histogram" );
      	  else
      	    cvNamedWindow( "Histogram", 1 );
      	  break;
      	default:
      	  ;
      	}
      
      cvCopy( image,frame, 0 );
      ////***

      
      if(++count_circle <= out_recordframeNO){
	if(count_circle == 1){
	  outTimeFile_circle<<"<header>"<<std::endl;
	  outTimeFile_circle<<"       <field>Index=1 type=\"Double\" name=\"timestamp\" description=\"the time since start of experiment\" unit=\"seconds\"</field>" << std::endl;
	  outTimeFile_circle<<"       <field>Index=2 type=\"Double\" name=\"x\" description=\"The x coordinate\" unit=\"meters\"</field>"<<std::endl;
	  outTimeFile_circle<<"       <field>Index=3 type=\"Double\" name=\"y\" description=\"The y coordinate\" unit=\"meters\"</field>"<<std::endl;
	  outTimeFile_circle<<"</header>"<<std::endl;
	  outTimeFile_circle<<"<data>"<<std::endl;	
	}

	//get the specific timestamp for each frame;
	string str_ts;
	int temp_frNO;
	int temp_ts;
	do{
	  getline(inTimeFile,str_ts);
	  stringstream ss(str_ts);
	  ss>>temp_frNO;
	  ss>>temp_ts;
	}while(temp_frNO != (srcFrame->getFrameNumber()+1)); //the first frame# begins from 0;

	outTimeFile_circle<<"  <row>  "<<temp_frNO<<"  1:"<<temp_ts<<"  2:"<<track_box.center.x<<"  3:"<<track_box.center.y<<"  </row>"<<std::endl;	    
	if( count_circle == out_recordframeNO )
	 outTimeFile_circle<<"</data>"<<std::endl;

	cvWriteFrame(writer_circle,frame);// add the frame to the file
      }
      ////////////////////////////////////////////////////////////////////
      cv::Mat out_prevBG(frame);//output the ''circle'' image;
      ////////////////////////////////////////////////////////////////////

      //store the result
      writepool->storeImage(out_prevBG, srcFrame->getFrameNumber());
   
      //set top criteria for the thread;
      if(count_circle == out_recordframeNO || (srcFrame->getFrameNumber()+1) >= in_videoframeNO){
	stopped = 1;
	std::cout<<"stopped thread!"<<std::endl;
      }
 
  
      // release reference counter after using frames
      srcFrame->release();
    }
    QThread::usleep(1000000 / 30);
  }
  ////////////////////////////////
   // cvDestroyWindow("CamShiftDemo");
  //cvDestroyWindow("111");
 // cvDestroyWindow("Histogram");

   //////////////////////////////
}
