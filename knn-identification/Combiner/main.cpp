#include <QtCore>
#include <QCoreApplication>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

QString prefix("../data/B144/Feeder_1.6M/");
int maxFrames = 22000;
string fileIDs = string("----,BBGY,BGBY,BPPO,")+
                 string("BPWP,GBGP,GO-B,GG-O,")+
                 string("GO-B,GOYP,GPBW,GW--,")+
                 string("GWBG,GWOW,GWPB,OGPG,")+
                 string("OOO-,OOOW,OP--,OPYO,")+
                 string("OWGW,P---,PGOP,PP--,")+
                 string("PYWB,WPWO,");
int frameAccumulator = 0;
int numAgents = fileIDs.length()/5;

bool processClip(int clipNum) {

    //Open the file
    QString fileName(prefix+"filtered"+QString::number(clipNum)+".csv");
    qDebug()<<"Opening"<<fileName;
    ifstream myfile1 (fileName.toUtf8().constData());
    if(!myfile1.is_open()) return false;

    //Initialize the business
    bool** antPresent = new bool*[maxFrames];
    for(int i=0; i<maxFrames; i++){
        antPresent[i] = new bool[numAgents];
        for(int j=0; j<numAgents; j++){
                antPresent[i][j]=false;
        }
    }

    //Read the file
    int maxFoundFrame = 0;
    string line;
    if(myfile1.good()) getline (myfile1,line); //the header line
    while(myfile1.good()){
            int frameNum, spotNum;
            char id[20]; //even 10 should be enough
            char commaTrash;
            myfile1 >> frameNum;
            myfile1 >> commaTrash;
            myfile1 >> spotNum;
            myfile1 >> commaTrash;
            myfile1 >> id;
            int idNum = fileIDs.find(string(id).substr(0,4));
            antPresent[frameNum-1][idNum/5] = true;
            if(frameNum > maxFoundFrame) maxFoundFrame = frameNum;
    }

    //Find events
    ofstream outFile;
    QString specifier = prefix;
    QString name = prefix+"combined"+specifier.remove(QChar('/')).remove(QChar('.'))+".csv";
    outFile.open(name.toUtf8().constData(), fstream::in | fstream::out | fstream::app);
    if(clipNum == 1) outFile << "Frame Number, ID, Action, \n";
    for(int i=1; i<maxFrames; i++){
        for(int j=0; j<numAgents; j++){
            if(antPresent[i][j] == true && antPresent[i-1][j] == false)
                outFile << (frameAccumulator+i) << "," << fileIDs.substr(j*5,4) << ", Enter, \n";
            else if(antPresent[i][j] == false && antPresent[i-1][j] == true)
                outFile << (frameAccumulator+i) << "," << fileIDs.substr(j*5,4) << ", Exit, \n";
        }
    }
    outFile.close();

    //Clean up
    myfile1.close();
    delete antPresent;
    frameAccumulator += maxFoundFrame;

    return true;
}

int main(int argc, char *argv[]) {

    int clip = 1;
    while(processClip(clip)) clip++;

    qDebug()<<"Done";
}
