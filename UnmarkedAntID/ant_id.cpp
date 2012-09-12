#include "ant_id.h"
#include "ui_ant_id.h"

Ant_ID::Ant_ID(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Ant_ID)
{
    ui->setupUi(this);
    K=3;

    tresholdValue=50;
    qDebug()<<"===========KNN Ant Identification Program===========\n( Starting with k ="<<K;

    traingingheader = "/home/blacki/Dropbox/Ants/knn/ant_Masked/";
    mugheader = "/home/blacki/Dropbox/Ants/knn/ant_Mugshots/";
    outheader = "/home/blacki/Dropbox/Ants/knn/Output_data/";

    qDebug()<<"Counting training images per ant...";
    QDir trainDir(traingingheader);
    QStringList filters;
    filters<<"ant*";
    trainDir.setNameFilters(filters);
    QStringList tfileList = trainDir.entryList(filters);
    qDebug()<<tfileList.size()<<" Ants in training set: "<< tfileList;


    QDir mugDir(mugheader);
    mugDir.setNameFilters(filters);
    QStringList mfileList = mugDir.entryList(filters);
    qDebug()<<mfileList.size()<<" Ants in unidentified set: "<< mfileList;

    numKnownAnts = mfileList.size();

    qDebug()<<numKnownAnts<<"ants exist";
    //Counting total number of training images
    numTrainingImgs = 0;
    QStringList::const_iterator iterator;
    for (iterator = tfileList.constBegin(); iterator != tfileList.constEnd(); ++iterator){
        QDir innerDir(traingingheader+(*iterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*org.JPG*"));
        QStringList::const_iterator innerIterator;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){
                    numTrainingImgs++;
        }
    }
    qDebug()<<"Total number of training images: "<<numTrainingImgs;

    //Counting total number of training images
    numUnknownImages =0;
    for (iterator = mfileList.constBegin(); iterator != mfileList.constEnd(); ++iterator){
        QDir innerDir(mugheader+(*iterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*.JPG*"));
        QStringList::const_iterator innerIterator;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){
            numUnknownImages++;
        }

    }

    qDebug()<<"Total number of Unknown Images"<<numUnknownImages;


    //Reading training images and getting max width and hieght of training images
    qDebug()<<"Processing training images...";
    qDebug()<<"     Converting Training images to YUV...";
    qDebug()<<"     Building Training histograms...";
    antIndex =0;
    QDir trainingDir(traingingheader);
    tfileList = trainingDir.entryList(filters);
    QStringList::const_iterator tIterator;

    maxTimgCols = 0;
    maxTimgRows = 0;

    namedWindow("Image");
    namedWindow( "U-V Histogram");


    for (tIterator = tfileList.constBegin(); tIterator != tfileList.constEnd(); ++tIterator){
        QDir innerDir(traingingheader+(*tIterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*org.JPG*"));
        //qDebug()<<innerFileList.size()<<"Org: "<<innerFileList;
        QStringList maskFileList = innerDir.entryList(QStringList("*_mask_ant.JPG*"));
        //qDebug()<<maskFileList.size()<<"mask: "<<maskFileList;



        QStringList::const_iterator innerIterator;
        int imageIndex = 0;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){
            //qDebug()<<(*tIterator);
            img = imread((traingingheader+(*tIterator)+"/"+(*innerIterator)).toUtf8().constData());
            if(!img.data){
                qDebug()<<"Could not load image file"<<(traingingheader+(*tIterator)+"/"+(*innerIterator));
            }
            else{

                maskedimg = Mat::zeros(img.size().width,img.size().height,CV_8UC3);

                mask = imread((traingingheader+(*tIterator)+"/"+(maskFileList.at(imageIndex))).toUtf8().constData());
                if(!mask.data){
                    qDebug()<<"Could not load mask file"<<(traingingheader+(*tIterator)+"/"+(maskFileList.at(imageIndex)));
                }

                maskgrey = cvCreateMat(mask.size().width, mask.size().height,CV_8UC1);

                cvtColor(mask,maskgrey, CV_RGB2GRAY);// rgb -> gray
                threshold(maskgrey,maskgrey,tresholdValue,255,CV_THRESH_BINARY); //threshold the gray
                img.copyTo(maskedimg,maskgrey);

                imshow("Image",maskedimg);

                hist = getYUVHist(maskedimg);
                trainingHistograms.push_back(hist);
                trainingIDs.push_back(antIndex);


                //sanity check compare against itself
                correl = compareHist(hist,hist,CV_COMP_CORREL);
                intersect = compareHist(hist,hist,CV_COMP_INTERSECT);

                    //U-V Histogram Image isn't working
                    imshow( "U-V Histogram", hist_img );
//                    qDebug()<<correl<<intersect;

                if (img.size().width > maxTimgCols){
                    maxTimgCols = img.size().width;
                }
                if (img.size().height > maxTimgRows){
                    maxTimgRows = img.size().height;
                }
               // qDebug()<< maxTimgCols << maxTimgRows;


                char c = cvWaitKey(10);
                if (c==27){
                    break;
                }

            }
            imageIndex++;
        }
        antIndex++;
    }



    qDebug()<<"Processing unidentified images...";

    qDebug()<<"     Converting to YUV...";
    qDebug()<<"     Building histograms...";

    //Unknown Images: Convert to YUV & Construct Histogram Array
    antIndex = 0;
    for (iterator = mfileList.constBegin(); iterator != mfileList.constEnd(); ++iterator){
        QDir innerDir(mugheader+(*iterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*.JPG*"));
        QStringList::const_iterator innerIterator;
        totalImages =0;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){
            img = imread((mugheader+(*iterator)+"/"+(*innerIterator)).toUtf8().constData());
            if(!img.data){
                qDebug()<<"Could not load image file"<<(mugheader+(*iterator)+"/"+(*innerIterator));
            }
            else{
                maskedimg = Mat::zeros(img.size().width,img.size().height,CV_8UC3);
                mask =imread((mugheader+"/bg/background1.jpg").toUtf8().constData());

                if(!mask.data){
                    qDebug()<<"Could not load mask file"<<mugheader+"/bg/background1.jpg";
                }

                maskgrey = cvCreateMat(mask.size().width, mask.size().height,CV_8UC1);
                Rect roi(0,0,img.size().width,img.size().height);
                maskedimg = mask(roi);

                absdiff(maskedimg,img,maskedimg);
                cvtColor(mask,maskgrey, CV_RGB2GRAY);// rgb -> gray
                threshold(maskgrey,maskgrey,tresholdValue,255,CV_THRESH_BINARY); //threshold the gray
                img.copyTo(maskedimg,maskgrey);

                imshow("Image",maskedimg);


                hist = getYUVHist(maskedimg);
                unknownHistograms.push_back(hist);

                //sanity check compare against itself
                correl = compareHist(hist,hist,CV_COMP_CORREL);
                intersect = compareHist(hist,hist,CV_COMP_INTERSECT);

                //U-V Histogram Image isn't working
                imshow( "U-V Histogram", hist_img );
//                qDebug()<<correl<<intersect;



                char c = cvWaitKey(10);
                if (c==27){
                    break;
                }

            }
//            qDebug()<<mfileList.at(antIndex)<<" "<<innerFileList.at(totalImages);
            totalImages++;
        }
        antIndex++;

    }


    qDebug()<<"Assigning IDs...";

    double **hypotheses = new double*[numUnknownImages]; //id and confidence
    for(int i=0; i<numUnknownImages; i++) hypotheses[i] = new double[2];
    double *averageDistance = new double[numUnknownImages];
    int agentVotes[numUnknownImages];
    for(int j=0; j<numUnknownImages; j++){
        agentVotes[j] = 0;
    }

    for(int i=0; i<numUnknownImages; i++){

        //find k nearest neighbors
              double nearestK[K][2];//id and confidence
              for(int k=0; k<K; k++){
                  nearestK[k][0] = -1;
                  nearestK[k][1] = 0;
              }
              for(int j=0; j<numTrainingImgs; j++){
                  correl = compareHist(unknownHistograms.at(i),trainingHistograms.at(j),CV_COMP_CORREL);
                  intersect = compareHist(unknownHistograms.at(i),trainingHistograms.at(j),CV_COMP_INTERSECT);

                  //qDebug()<<correl<<intersect;
//                  double similarity = intersect;
                  double similarity = correl;

                  int furthestNeighbor = 0;
                  for(int k=1; k<K; k++){
                      if(nearestK[k][1]<nearestK[furthestNeighbor][1])
                          furthestNeighbor = k;
                  }
                  if(similarity > nearestK[furthestNeighbor][1]){
                      nearestK[furthestNeighbor][1] = similarity;
                      nearestK[furthestNeighbor][0] = trainingIDs.at(j);
                  }
              }
              //poll the neighbors
              int agentVotes[numKnownAnts];
              for(int j=0; j<numKnownAnts; j++){
                  agentVotes[j] = 0;
              }

              for(int k=0; k<K; k++){
                  agentVotes[(int)(nearestK[k][0]-1)]++;
              }
              int majorityVote = 0;
              //qDebug()<<agentVotes[0];
              for(int j=0; j<numKnownAnts; j++){
                  if(agentVotes[j]>agentVotes[majorityVote])
                      majorityVote = j;
                  qDebug()<<agentVotes[j];
              }
              qDebug()<<"--";
              hypotheses[i][0] = majorityVote+1;//this 'sometimes zero-indexed
              hypotheses[i][1] = ((double)agentVotes[majorityVote])/K;
              averageDistance[i] = 0;
              for(int k=0; k<K; k++){
                  //if((int)(nearestK[k][0]) == majorityVote)
                      averageDistance[i]+=nearestK[k][1];
              }
              averageDistance[i] /= K;//((double)agentVotes[majorityVote]);
    }


    ofstream myFile;
    name = outheader+"results.csv";
    myFile.open(name.toUtf8().constData());
    myFile << "Image, ID, Hypotheses, Confidence, Similarity, \n";


    antIndex = 0;
    totalImages =0;
    numMatch=0;
    for (iterator = mfileList.constBegin(); iterator != mfileList.constEnd(); ++iterator){
        QDir innerDir(mugheader+(*iterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*.JPG*"));
        QStringList::const_iterator innerIterator;
        int imageIndex = 0;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){


            if(mfileList.at(antIndex).toStdString()==mfileList.at(hypotheses[imageIndex][0]).toStdString()){
                match = 1;
                numMatch++;
            }
          //  qDebug()<<"mfileList"<<mfileList;
          //  qDebug()<<"numMatch"<<numMatch;
            //totalImages++;
            myFile << innerFileList.at(imageIndex).toStdString() << "," << mfileList.at(antIndex).toStdString()<< "," <<mfileList.at(hypotheses[antIndex+imageIndex][0]).toStdString()<< "," << hypotheses[antIndex+imageIndex][1] << "," << averageDistance[antIndex+imageIndex] << ", \n";
                match = 0;
            //qDebug()<<totalImages<<mfileList.at(antIndex)<<mfileList.at(hypotheses[totalImages][0]);
            imageIndex++;
            totalImages++;
        }
        antIndex++;

    }
    qDebug()<< "Percent matched: "<< (double)numMatch/numUnknownImages*100 <<"%, k:"<<K<<",n"<<numUnknownImages;
    myFile << "Percent matched: ,"<< (double)numMatch/numUnknownImages*100 <<"%, k:"<<K<<",n"<<numUnknownImages;
    myFile.close();
    qDebug()<<"Output saved to"<<name;
}

Ant_ID::~Ant_ID()
{
    delete ui;
}

MatND Ant_ID::getYUVHist(Mat src)
{
//    Mat y_plane = cvCreateMat(src.size().width, src.size().height, CV_8UC1);
//    Mat u_plane = cvCreateMat(src.size().width, src.size().height, CV_8UC1);
//    Mat v_plane = cvCreateMat(src.size().width, src.size().height, CV_8UC1);
//    Mat planes [] = {y_plane,u_plane};
    Mat yuv1 = cvCreateMat (src.size().width, src.size().height,CV_8UC3);

    int y_bins =32, u_bins = 32;
    int hist_size [] = {y_bins, u_bins};
    float y_ranges[]={0,255};
    float u_ranges []={0,255};
    const float * ranges[] = {y_ranges, u_ranges};

    MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    int scale = 10;
    hist_img = Mat::zeros(y_bins*scale,u_bins*scale,CV_8UC3);

    double max_value = 0;
    int y,u;


    cvtColor( src, yuv1, CV_RGB2YCrCb );
    calcHist( &yuv1, 1, channels, Mat(), // do not use mask
             hist, 2, hist_size, ranges,
             true, // the histogram is uniform
             false );

    minMaxLoc(hist, 0, &max_value, 0, 0);

    for (y=0; y<y_bins; y++){
        for (u=0; u<u_bins; u++){
            float binVal = hist.at<float>(y, u);
            int intensity = cvRound(binVal*255/max_value);
            rectangle( hist_img, cvPoint( y*scale, u*scale ),
                cvPoint( (y+1)*scale - 1, (u+1)*scale - 1),
                CV_RGB(intensity,intensity,intensity),
                CV_FILLED );
        }
    }

    normalize(hist, hist, 1, 0, NORM_L1);
    return (hist);
}
