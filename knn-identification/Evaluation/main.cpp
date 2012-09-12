#include <QtCore>
#include <QCoreApplication>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int numFrames = 22000;//21216;
int numAgents = 18;
string file2ids("BBGY,BGBY,BPPO,BPWP,GBGP,GO*B,GOYP,GPBW,GW**,GWBG,GWOW,GWPB,OGPG,OOOW,OP**,OPYO,PGOP,PP**,GOB*");
string file3ids("BBGY,BGBY,BPPO,BPWP,GBGP,GO/B,GOYP,GPBW,GW//,GWBG,GWOW,GWPB,OGPG,OOOW,OP//,OPYO,PGOP,PP//,GOB/");
int gracePeriod = 150; //30 frames is 1 second of "forgiveness time"

//Frame-level metric
void compareFrames(bool** file1, bool** file2){
	int numCorrect = 0, file1Only = 0, file2Only = 0;
	double* agentCorrect = new double[numAgents];
	for(int i=0; i<numAgents; i++){
		agentCorrect[i] = 0;
	}
	for(int i=0; i<numFrames; i++){
		for(int j=0; j<numAgents; j++){
			bool correct = false;
			for(int k=i-gracePeriod; k<=i+gracePeriod; k++){
				if(k >= 0 && k < numFrames){
					if((file1[k][j] && file2[k][j]) || (!file2[k][j] && !file1[k][j])){
						correct = true;
					}
					else if(file1[k][j] && !file2[k][j])
						file1Only++;
					else if(!file1[k][j] && file2[k][j])
						file2Only++;
				}
			}
			if(correct){
				agentCorrect[j]++;
				numCorrect++;
			}
		}
	}
	qDebug()<<"Frame by Frame comparison ("<<gracePeriod<<"frames of flexibility. )";
	qDebug()<<"True positive matches:";
	qDebug()<<" "<<100*(double)numCorrect/(numFrames*numAgents)<<"%";
	qDebug()<<"File 1 true, but File 2 false:";
	qDebug()<<" "<<100*(1-(double)numCorrect/(numFrames*numAgents))*(double)file1Only/(file1Only+file2Only)<<"%";
	qDebug()<<"File 2 true, but File 1 false:";
	qDebug()<<" "<<100*(1-(double)numCorrect/(numFrames*numAgents))*(double)file2Only/(file1Only+file2Only)<<"%";
	qDebug()<<"Individual Results:";
	for(int i=0; i<numAgents; i++)
		qDebug()<<" "<<i<<":"<<agentCorrect[i]<<"correct out of"<<numFrames<<"("<<100*(double)agentCorrect[i]/numFrames<<"% ).";
	delete agentCorrect;
}

//Flexible event-level metric
void compareEvents(bool** file1, bool** file2){
	qDebug()<<"Event level comparison with window of"<<gracePeriod<<":";
	int numCorrect = 0, totalEvents = 0;
	for(int start=1; start<numFrames; start++){
		for(int j=0; j<numAgents; j++){
			if(file1[start][j] == true && file1[start-1][j] == false){ //start event in file 1
				totalEvents++;
				int end = start;
				for(int i=start-gracePeriod; i<end+gracePeriod; i++){
					if(i < 0 || i >= numFrames) continue;
					if(file1[start][j] == file1[i][j])
						end++;
					if(file2[i][j] == file1[start][j]){ //"matching" event found in file 2
						numCorrect++;
						break;
					}
				}
			}
		}
	}
	qDebug()<<"Finding matching events in file 2 for every event in file 1:";
	qDebug()<<totalEvents<<"events found,"<<numCorrect<<"correct ("<<100*(double)numCorrect/totalEvents<<"% ).";
	//qDebug()<<totalEvents<<"+"<<numFalse<<"+"<<numTrue<<"="<<(totalEvents+numFalse+numTrue);
	numCorrect = 0, totalEvents = 0;
	for(int start=1; start<numFrames; start++){
		for(int j=0; j<numAgents; j++){
			if(file2[start][j] == true && file2[start-1][j] == false){ //start event in file 2
				totalEvents++;
				int end = start;
				for(int i=start-gracePeriod; i<end+gracePeriod; i++){
					if(i < 0 || i >= numFrames) continue;
					if(file2[start][j] == file2[i][j])
						end++;
					if(file1[i][j] == file2[start][j]){ //"matching" event found in file 1
						numCorrect++;
						break;
					}
				}
			}
		}
	}
	qDebug()<<"Finding matching events in file 1 for every event in file 2:";
	qDebug()<<totalEvents<<"events found,"<<numCorrect<<"correct ("<<100*(double)numCorrect/totalEvents<<"% ).";
	//qDebug()<<totalEvents<<"+"<<numFalse<<"+"<<numTrue<<"="<<(totalEvents+numFalse+numTrue);
}

////"Harsh" start-to-end evaluation metric
//void compareEvents(bool** file1, bool** file2){
//	qDebug()<<"Event level comparison with window of"<<gracePeriod<<":";
//	int numCorrect = 0, totalEvents = 0, numFalse = 0, numTrue = 0;
//	for(int i=1; i<numFrames; i++){
//		for(int j=0; j<numAgents; j++){
//			if(file1[i][j] == true && file1[i-1][j] == false){ //start event in file 1
//				totalEvents++;
//				for(int k=i-gracePeriod; k<=i+gracePeriod; k++){
//					if(k >= 1 && k < numFrames){
//						if(file2[k][j] == true && file2[k-1][j] == false){ //matches a start event in file 2
//							numCorrect++;
//							break;
//						}
//					}
//				}
//			}
//			else if(file1[i][j] == false && file1[i-1][j] == true){ //end event in file 1
//				totalEvents++;
//				for(int k=i-gracePeriod; k<=i+gracePeriod; k++){
//					if(k >= 1 && k < numFrames){
//						if(file2[k][j] == false && file2[k-1][j] == true){ //matches an end event in file 2
//							numCorrect++;
//							break;
//						}
//					}
//				}
//			}
//			else if(file1[i][j] == false && file1[i-1][j] == false) numFalse++;
//			else if(file1[i][j] == true && file1[i-1][j] == true) numTrue++;
//		}
//	}
//	qDebug()<<"Finding matching events in file 2 for every event in file 1:";
//	qDebug()<<totalEvents<<"events found,"<<numCorrect<<"correct ("<<100*(double)numCorrect/totalEvents<<"% ).";
//	//qDebug()<<totalEvents<<"+"<<numFalse<<"+"<<numTrue<<"="<<(totalEvents+numFalse+numTrue);
//	numCorrect = 0; totalEvents = 0; numFalse = 0; numTrue = 0;
//	for(int i=1; i<numFrames; i++){
//		for(int j=0; j<numAgents; j++){
//			if(file2[i][j] == true && file2[i-1][j] == false){ //start event in file 2
//				totalEvents++;
//				for(int k=i-gracePeriod; k<=i+gracePeriod; k++){
//					if(k >= 1 && k < numFrames){
//						if(file1[k][j] == true && file1[k-1][j] == false){ //matches a start event in file 1
//							numCorrect++;
//							break;
//						}
//					}
//				}
//			}
//			else if(file2[i][j] == false && file2[i-1][j] == true){ //end event in file 2
//				totalEvents++;
//				for(int k=i-gracePeriod; k<=i+gracePeriod; k++){
//					if(k >= 1 && k < numFrames){
//						if(file1[k][j] == false && file1[k-1][j] == true){ //matches an end event in file 1
//							numCorrect++;
//							break;
//						}
//					}
//				}
//			}
//			else if(file2[i][j] == false && file2[i-1][j] == false) numFalse++;
//			else if(file2[i][j] == true && file2[i-1][j] == true) numTrue++;
//		}
//	}
//	qDebug()<<"Finding matching events in file 1 for every event in file 2:";
//	qDebug()<<totalEvents<<"events found,"<<numCorrect<<"correct ("<<100*(double)numCorrect/totalEvents<<"% ).";
//	//qDebug()<<totalEvents<<"+"<<numFalse<<"+"<<numTrue<<"="<<(totalEvents+numFalse+numTrue);
//}

void evaluateClip(int clipNum) {

    qDebug()<<"\nClip:"<<clipNum;

    bool** file1 = new bool*[numFrames];
    bool** file2 = new bool*[numFrames];
    bool** file3 = new bool*[numFrames];
    for(int i=0; i<numFrames; i++){
        file1[i] = new bool[numAgents];
        file2[i] = new bool[numAgents];
        file3[i] = new bool[numAgents];
        for(int j=0; j<numAgents; j++){
                file1[i][j]=false;
                file2[i][j]=false;
                file3[i][j]=false;
        }
    }

        string line;
        //1
    ifstream myfile1 (QString("../data/filtered"+QString::number(clipNum)+".csv").toUtf8().constData());
        if(!myfile1.is_open()) exit(0);
        if(myfile1.good()) getline (myfile1,line); //the header line
        while(myfile1.good()){
                int frameNum, spotNum, id;
                double trash, similarity;
                char commaTrash;
                myfile1 >> frameNum;
                myfile1 >> commaTrash;
                myfile1 >> spotNum;
                myfile1 >> commaTrash;
                myfile1 >> id;
                myfile1 >> commaTrash;
                myfile1 >> trash;
                myfile1 >> commaTrash;
                myfile1 >> similarity;
                myfile1 >> commaTrash;
                if(frameNum>=2000)//as an artifact of the "break into clips process, ants currently feeding at the start of a clip are lost in human data
                file1[frameNum-1][id-1] = true;
        }
        //2
        ifstream myfile2 (QString("../data/B144_datalog_0.1M_Parsed (Clip "+QString::number(clipNum)+").csv").toUtf8().constData());
        if(!myfile2.is_open()) exit(0);
        if(myfile2.good()) getline (myfile2,line); //the header line
        while(myfile2.good()){
                int frameNum;
                string frame,id,trash;
                getline(myfile2,frame,',');//frame
                getline(myfile2,id,',');//id
                getline(myfile2,trash);//everything else
                frameNum = atoi(frame.c_str());
                size_t idNum = file2ids.find(id) / 5;
                if(idNum > 18) cout<<"Undefined agent found in file 2: "<<id<<endl;
                else if(idNum == 18) idNum = 5;
                if(idNum > 0 && idNum < 18) file2[frameNum-1][int(idNum)] = true;
        }
//        //3
//        ifstream myfile3 (QString("../data/B144_human2_datalog_0.1M_Parsed (Clip "+QString::number(clipNum)+").csv").toUtf8().constData());
//        if(!myfile3.is_open()) exit(0);
//        if(myfile3.good()) getline (myfile3,line); //the header line
//        while(myfile3.good()){
//                int frameNum;
//                string frame,id,trash;
//                getline(myfile3,frame,',');//frame
//                getline(myfile3,id,',');//id
//                getline(myfile3,trash);//everything else
//                frameNum = atoi(frame.c_str());
//                size_t idNum = file3ids.find(id) / 5;
//                if(idNum > 18) cout<<"Undefined agent found in file 3: "<<id<<endl;
//                else if(idNum == 18) idNum = 5;
//                if(idNum > 0 && idNum < 18) file3[frameNum-1][int(idNum)] = true;
//        }

        qDebug()<<"\n\nFrames:\n";
        qDebug()<<"\nComputer to Human 1:\n";
        compareFrames(file1,file2);
//        qDebug()<<"\nComputer to Human 2:\n";
//        compareFrames(file1,file3);
//        qDebug()<<"\nHuman 1 to Human 2:\n";
//        compareFrames(file2,file3);

        qDebug()<<"\n\nEvents:\n";
        qDebug()<<"\nComputer to Human 1:\n";
        compareEvents(file1,file2);
//        qDebug()<<"\nComputer to Human 2:\n";
//        compareEvents(file1,file3);
//        qDebug()<<"\nHuman 1 to Human 2:\n";
//        compareEvents(file2,file3);

        delete file1;
        delete file2;
        delete file3;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    for(int clip=6; clip<=8; clip++) //don't have human data for all clips
        evaluateClip(clip);

    return a.exec();
}
