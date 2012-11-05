#include "main.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    QString activeFolder = QString("/B144/Feeder_1.6M/");

    int clipNum = 0;
    QString header = "/media/8865399a-a349-43cd-8cc0-2b719505efaf"+activeFolder;
    QDir unkDir(header);
    QStringList fileList = unkDir.entryList(QStringList("FeederMugshots_*"));
    QStringList::const_iterator unkIterator;
    for (unkIterator = fileList.constBegin(); unkIterator != fileList.constEnd(); ++unkIterator){
        clipNum++;
        identifyClip(activeFolder+(*unkIterator)+"/", clipNum);
    }

    return a.exec();
}

void identifyClip(QString activeFolder, int clipNum){
    qDebug()<<"Clip"<<clipNum<<":"<<activeFolder;

    qDebug()<<"===========KNN Ant Identification Program===========\n( Starting with k ="<<K<<"bgSimilarity ="<<bgSimilarity<<"binSize ="<<binSize<<")";

    qDebug()<<"Counting ants...";
    QString header = "../data/KNNtraining/";
    QDir preDir(header);
    QStringList fileList = preDir.entryList(QStringList("????"));
    int numKnownAnts = fileList.length();
    int maxSamplesPerAnt = 0;
    QStringList::const_iterator iterator;
    for (iterator = fileList.constBegin(); iterator != fileList.constEnd(); ++iterator){
        QDir innerDir(header+(*iterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*.png"));
        if(innerFileList.length()>maxSamplesPerAnt) maxSamplesPerAnt = innerFileList.length();
    }

    qDebug()<<"Initializing data structures...";
    RgbImage **RGBsamples = new RgbImage*[numKnownAnts];
    for(int i=0 ; i<numKnownAnts ; i++){ RGBsamples[i] = new RgbImage[maxSamplesPerAnt]; }
    const int numUnknownImages = maxFrames * numParkingSpots;
    bool *nullPtr = new bool[numUnknownImages];
    UVHistogram *H = new UVHistogram[numKnownAnts*maxSamplesPerAnt];
    UVHistogram *unknownH = new UVHistogram[numUnknownImages];
    YuvPixel ****YUVsamples = new YuvPixel***[numKnownAnts];
    for(int i=0 ; i<numKnownAnts ; i++){
    	YUVsamples[i] = new YuvPixel**[maxSamplesPerAnt];
        for(int j=0 ; j<maxSamplesPerAnt ; j++){
            YUVsamples[i][j] = new YuvPixel*[maxImageRows];
            for(int k=0; k<maxImageRows; k++){
                YUVsamples[i][j][k] = new YuvPixel[maxImageCols];
            }
        }
    }
    int *samplesPerAnt = new int[numKnownAnts];

    qDebug()<<"Reading training images...";
    header = "../data/KNNtraining/";
    QDir trainingDir(header);
    fileList = trainingDir.entryList(QStringList("????"));
    qDebug()<<fileList;
    QStringList::const_iterator tIterator;
    int antIndex = 0;
    for (tIterator = fileList.constBegin(); tIterator != fileList.constEnd(); ++tIterator){
        QDir innerDir(header+(*tIterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*.png"));
        QStringList::const_iterator innerIterator;
        int imageIndex = 0;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){
            IplImage* img = 0;
            img = cvLoadImage((header+(*tIterator)+"/"+(*innerIterator)).toUtf8().constData());
            if(!img){
                qDebug()<<"Could not load image file"<<(header+(*tIterator)+"/"+(*innerIterator));
            }
            else{
                RgbImage rgbImg(img);
                RGBsamples[antIndex][imageIndex] = rgbImg;
                samplesPerAnt[antIndex] = imageIndex+1;
            }
            imageIndex++;
        }
        antIndex++;
    }

    qDebug()<<"Converting to YUV...";
    for(int i=1; i<=numKnownAnts; i++){
        for(int j=1; j<=samplesPerAnt[i-1]; j++){
            for(int r=0; r<RGBsamples[i-1][j-1].height(); r++){
                for(int c=0; c<RGBsamples[i-1][j-1].width(); c++){
                    double Y = 0.299*RGBsamples[i-1][j-1][r][c].r + 0.587*RGBsamples[i-1][j-1][r][c].g + 0.114*RGBsamples[i-1][j-1][r][c].b;
                    double U = (RGBsamples[i-1][j-1][r][c].b - Y)*0.565;
                    double V = (RGBsamples[i-1][j-1][r][c].r - Y)*0.713;
                    YUVsamples[i-1][j-1][r][c].y = Y;
                    YUVsamples[i-1][j-1][r][c].u = U;
                    YUVsamples[i-1][j-1][r][c].v = V;
                }
            }
        }
    }

    qDebug()<<"Building histograms...";
    for(int i=1; i<=numKnownAnts; i++){
        for(int j=1; j<=samplesPerAnt[i-1]; j++){
            H[(i-1)*maxSamplesPerAnt+j-1].agentId = i;
            for(int x=0; x<256; x++){
                H[(i-1)*maxSamplesPerAnt+j-1].UValues[x] = 0;
                H[(i-1)*maxSamplesPerAnt+j-1].VValues[x] = 0;
            }
            for(int r=0; r<RGBsamples[i-1][j-1].height(); r++){
                for(int c=0; c<RGBsamples[i-1][j-1].width(); c++){
                    if(!(similar(0, YUVsamples[i-1][j-1][r][c].u, bgSimilarity) && similar(0, YUVsamples[i-1][j-1][r][c].v, bgSimilarity))){
                        H[(i-1)*maxSamplesPerAnt+j-1].UValues[(YUVsamples[i-1][j-1][r][c].u + 128)/binSize]++;
                        H[(i-1)*maxSamplesPerAnt+j-1].VValues[(YUVsamples[i-1][j-1][r][c].v + 128)/binSize]++;
                    }
                }
            }
            H[(i-1)*maxSamplesPerAnt+j-1].normalize();
        }
        for(int j=samplesPerAnt[i-1]+1; j<=maxSamplesPerAnt; j++){
            for(int x=0; x<256; x++){
                H[(i-1)*maxSamplesPerAnt+j-1].UValues[x] = 0;
                H[(i-1)*maxSamplesPerAnt+j-1].VValues[x] = 0;
            }
        }
    }
    delete [] RGBsamples;
    delete [] YUVsamples;

    qDebug()<<"Processing unidentified images...";
    header = "/media/8865399a-a349-43cd-8cc0-2b719505efaf"+activeFolder;

    for(int i=0; i<maxFrames; i++){
        for(int j=0; j<numParkingSpots; j++){
            nullPtr[(i)*numParkingSpots + j ] = true;
            unknownH[(i)*numParkingSpots + j ].agentId = -1;
            for(int x=0; x<256; x++){
                unknownH[(i)*numParkingSpots + j ].UValues[x] = 0;
                unknownH[(i)*numParkingSpots + j ].VValues[x] = 0;
            }
        }
    }
    QDir unknownDir(header);
    fileList = unknownDir.entryList(QStringList("pSpot*"));
    QStringList::const_iterator uIterator;
    for (uIterator = fileList.constBegin(); uIterator != fileList.constEnd(); ++uIterator){
        qDebug()<<"  Beginning images in"<<(*uIterator);
        QDir innerDir(header+(*uIterator)+"/");
        QStringList innerFileList = innerDir.entryList(QStringList("*.png"));
        QStringList::const_iterator innerIterator;
        for (innerIterator = innerFileList.constBegin(); innerIterator != innerFileList.constEnd(); ++innerIterator){
            IplImage* img = 0;
            img = cvLoadImage((header+(*uIterator)+"/"+(*innerIterator)).toUtf8().constData());
            if(!img){
                 qDebug()<<"Could not load image file"<<(header+(*uIterator)+"/"+(*innerIterator));
            }
            else{
                RgbImage rgbImg(img);
                QString name = (*innerIterator);
                name.remove(QString("framenum_"));
                name.remove(QString(".png"));
                //QStringList parts = name.split("_");
                //int i = parts[3].toInt();//frame
                //int j = parts[0].toInt();//spot
                int i = name.toInt();
                QString spotName = (*uIterator);
                int j = spotName.remove("pSpot").toInt();
                nullPtr[(i)*numParkingSpots + j ] = false;
                for(int r=0; r<rgbImg.height(); r++){
                    for(int c=0; c<rgbImg.width(); c++){
                        double Y = 0.299*rgbImg[r][c].r + 0.587*rgbImg[r][c].g + 0.114*rgbImg[r][c].b;
                        double U = (rgbImg[r][c].b - Y)*0.565;
                        double V = (rgbImg[r][c].r - Y)*0.713;
                        if(!(similar(0, ((int)U), bgSimilarity) && similar(0, ((int)V), bgSimilarity))){
                            unknownH[(i)*numParkingSpots + j ].UValues[(((int)U) + 128)/binSize]++;
                            unknownH[(i)*numParkingSpots + j ].VValues[(((int)V) + 128)/binSize]++;
                        }
                    }
                }
                unknownH[(i)*numParkingSpots + j ].normalize();
            }
            cvReleaseImage(&img);
        }
    }

//	for(int i=1; i<=maxFrames; i++){
//		for(int j=1; j<=numParkingSpots; j++){
//			QString name, fileName;
//			IplImage* img=0;
//			//name = "clipnum_"+QString::number(3)+"_framenum_"+QString::number(i)+"_spotnum_"+QString::number(j)+".png";
//			name = parkingSpotNames[j-1]+QString::number(i)+".png";
//			fileName = header+name;
//			img=cvLoadImage(fileName.toUtf8().constData());
//			if(!img){
//				nullPtr[(i-1)*numParkingSpots + j - 1] = true;
//			}
//			else{
//				RgbImage rgbImg(img);
//				unknowns[(i-1)*numParkingSpots + j - 1] = rgbImg;
//				nullPtr[(i-1)*numParkingSpots + j - 1] = false;
//			}
//			unknownH[(i-1)*numParkingSpots + j - 1].agentId = -1;
//			for(int x=0; x<256; x++){
//				unknownH[(i-1)*numParkingSpots + j - 1].UValues[x] = 0;
//				unknownH[(i-1)*numParkingSpots + j - 1].VValues[x] = 0;
//			}
//			if(nullPtr[(i-1)*numParkingSpots + j - 1]){
//				continue;
//			}
//			for(int r=0; r<unknowns[(i-1)*numParkingSpots + j - 1].height(); r++){
//				for(int c=0; c<unknowns[(i-1)*numParkingSpots + j - 1].width(); c++){
//					double Y = 0.299*unknowns[(i-1)*numParkingSpots + j - 1][r][c].r + 0.587*unknowns[(i-1)*numParkingSpots + j - 1][r][c].g + 0.114*unknowns[(i-1)*numParkingSpots + j - 1][r][c].b;
//					double U = (unknowns[(i-1)*numParkingSpots + j - 1][r][c].b - Y)*0.565;
//					double V = (unknowns[(i-1)*numParkingSpots + j - 1][r][c].r - Y)*0.713;
//					if(!(similar(0, ((int)U), bgSimilarity) && similar(0, ((int)V), bgSimilarity))){
//						unknownH[(i-1)*numParkingSpots + j - 1].UValues[(((int)U) + 128)/binSize]++;
//						unknownH[(i-1)*numParkingSpots + j - 1].VValues[(((int)V) + 128)/binSize]++;
//					}
//				}
//			}
//			unknownH[(i-1)*numParkingSpots + j - 1].normalize();
//
//			cvReleaseImage(&img);
//
//		}
//		if(i%1000==0)
//			qDebug()<<"( Frame"<<i<<")";
//	}
//	delete [] unknowns;

    header = "../data"+activeFolder;
    QDir dir(header); if (!dir.exists()) dir.mkpath(".");

    qDebug()<<"Computing confusion matrix...";
    int confHeight = 480, confWidth = 2*confHeight;//, buffer = 2, unknownWidth = (double)(confWidth/(numKnownAnts*maxSamplesPerAnt))*numUnknownImages;
    QString name = header+"confusionmat"+QString::number(clipNum)+".png";
    //cvNamedWindow("ConfusionMatrix", CV_WINDOW_AUTOSIZE);
    IplImage* confImg = cvCreateImage(cvSize(confWidth,confHeight), IPL_DEPTH_8U, 1);
    BwImage confMat(confImg);
    int totalUnknownSamples = 0;
    for(int i=1; i<=numUnknownImages; i++){
        if(nullPtr[i-1]){
            continue;
        }
        totalUnknownSamples++;
    }
    int totalKnownSamples = 0;
    for(int i=0; i<numKnownAnts;i++)
        totalKnownSamples += samplesPerAnt[i];
    int vertStep = max(confHeight/totalKnownSamples, 1);
    int horzStep = max((confWidth/2)/totalKnownSamples, 1);
    int stepRow = 0;
    for(int i=1; i<=numKnownAnts; i++){
        for(int j=1; j<=samplesPerAnt[i-1]; j++){
            int rowIndex = (i-1)*maxSamplesPerAnt+j-1;
            int stepCol = 0;
            for(int ii=1; ii<=numKnownAnts; ii++){
                for(int jj=1; jj<=samplesPerAnt[ii-1]; jj++){
                    int colIndex = (ii-1)*maxSamplesPerAnt+jj-1;
                    for(int k=0; k<=vertStep; k++){
                        for(int kk=0; kk<=horzStep; kk++){
                            confMat[min(confHeight,(int)(((double)stepRow/totalKnownSamples)*confHeight+k))]
                                   [min(confWidth/2, (int)(((double)stepCol/totalKnownSamples)*(confWidth/2)+kk))] = 255 * H[rowIndex].intersectionWith(H[colIndex]);
                        }
                    }
                    stepCol++;
                }
            }
            stepCol = 0;
            for(int ii=1; ii<=maxFrames; ii++){
                for(int jj=1; jj<=numParkingSpots; jj++){
                    int colIndex = (ii-1)*numParkingSpots + jj - 1;
                    if(!nullPtr[colIndex]){
                        for(int k=0; k<=vertStep; k++)
                            confMat[min(confHeight,(int)(((double)stepRow/totalKnownSamples)*confHeight+k))]
                                   [confWidth/2+(int)(((double)stepCol/totalUnknownSamples)*(confWidth/2))] = 255 * H[rowIndex].intersectionWith(unknownH[colIndex]);
                        stepCol++;
                    }
                }
            }
            stepRow++;
        }
    }
    //cvShowImage("ConfusionMatrix", confImg);
    cvSaveImage(name.toUtf8().constData(),confImg);

    qDebug()<<"Assigning IDs...";
    double **hypotheses = new double*[numUnknownImages]; //id and confidence
    for(int i=0; i<numUnknownImages; i++) hypotheses[i] = new double[2];
    double *averageDistance = new double[numUnknownImages];
    for(int i=0; i<numUnknownImages; i++){
        if(nullPtr[i]){
            continue;
        }
        //find k nearest neighbors
        double nearestK[K][2];//id and confidence
        for(int k=0; k<K; k++){
            nearestK[k][0] = -1;
            nearestK[k][1] = 0;
        }
        for(int j=0; j<numKnownAnts*maxSamplesPerAnt; j++){
            double similarity = unknownH[i].intersectionWith(H[j]);
            int furthestNeighbor = 0;
            for(int k=1; k<K; k++){
                if(nearestK[k][1]<nearestK[furthestNeighbor][1])
                    furthestNeighbor = k;
            }
            if(similarity > nearestK[furthestNeighbor][1]){
                nearestK[furthestNeighbor][1] = similarity;
                nearestK[furthestNeighbor][0] = H[j].agentId;
            }
        }
        //poll the neighbors
        int agentVotes[numKnownAnts];
        for(int j=0; j<numKnownAnts; j++){agentVotes[j] = 0;}
        for(int k=0; k<K; k++){agentVotes[(int)(nearestK[k][0]-1)]++;}
        int majorityVote = 0;
        //qDebug()<<agentVotes[0];
        for(int j=1; j<numKnownAnts; j++){
            if(agentVotes[j]>agentVotes[majorityVote])
                majorityVote = j;
            //qDebug()<<agentVotes[j];
        }
        //qDebug()<<"--";
        hypotheses[i][0] = majorityVote+1;//this 'sometimes zero-indexed, sometimes one-indexed' business is going to bite us later
        hypotheses[i][1] = ((double)agentVotes[majorityVote])/K;
        averageDistance[i] = 0;
        for(int k=0; k<K; k++){
            //if((int)(nearestK[k][0]) == majorityVote)
                averageDistance[i]+=nearestK[k][1];
        }
        averageDistance[i] /= K;//((double)agentVotes[majorityVote]);
    }
    ofstream myFile;
    name = header+"results"+QString::number(clipNum)+".csv";
    myFile.open(name.toUtf8().constData());
    myFile << "Frame Number, Spot Number, ID, Confidence, Similarity, \n";
    for(int i=0; i<numUnknownImages; i++){
        if(nullPtr[i]){
            continue;
        }
        //qDebug()<<"Image"<<i+1<<"is of agent"<<hypotheses[i][0]<<"("<<hypotheses[i][1]*100<<"% agree at"<<averageDistance[i]<<")";
        //if(averageDistance[i]>=0.9){
            myFile << ((i/numParkingSpots) + 1) << "," << ((i%numParkingSpots) + 1) << "," << hypotheses[i][0] << "," << hypotheses[i][1] << "," << averageDistance[i] << ", \n";
        //}
    }
    myFile.close();
    qDebug()<<"Output saved to"<<name;

    delete [] averageDistance;
    delete [] samplesPerAnt;
    delete [] hypotheses;
    delete [] unknownH;
    delete [] nullPtr;
    delete [] H;

    qDebug()<<"=====================Clean Exit=====================";
}
