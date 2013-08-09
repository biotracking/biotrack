#include <QtGui/QApplication>
#include "BackgroundCalculator.h"
#include "BackgroundCalculatorAverage.h"
#include "BackgroundCalculatorMode.h"
#include "backgrounder.h"
#include <cstdio>
#include <string>
/***

Copyright 2012 Andrew Quitmeyer and Georgia Tech's Multi Agent Robotics and Systems Lab

    The files included with this piece of software are free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


***/
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Backgrounder w;
    if(argc<=1){
        w.show();
        return a.exec();
    }else{

        if(argc<3){
            cout << "Backgrounder - computes the video's' background image" << endl;
            cout << endl;
            cout << "usage: Backgrounder VIDEO_FILE OUTPUT_FILE [average|mode|median]" << endl;
            cout << endl;
            cout << "Program computes the VIDEO_FILE's background image using average (default), mode or median of frames pixels, and saves it to the OUTPUT_FILE." << endl;
            cout << endl;
            return 0;
        }

        QString videopath = QString(argv[1]);
        VideoCapture capture;
        capture.open(videopath.toStdString());
        int frames=capture.get(CV_CAP_PROP_FRAME_COUNT)-1;

        BackgroundCalculator* backgroundCalculator = new BackgroundCalculatorAverage(&capture,0,frames );

        if(argc==4){
            if(QString(argv[3]) == QString("mode")){
                delete(backgroundCalculator);
                backgroundCalculator = new BackgroundCalculatorMode(&capture,0,frames,Mode);
                cout << "Using MODE mode" << endl;
            }else if(QString(argv[3]) == QString("median")){
                delete(backgroundCalculator);
                backgroundCalculator = new BackgroundCalculatorMode(&capture,0,frames,Median);
                cout << "Using MEDIAN mode" << endl;
            }else cout << "Using AVERAGE mode" << endl;
        }else cout << "Using AVERAGE mode" << endl;
        cout << "Starting background calulcation" << endl;

        do{
            backgroundCalculator->step();
        }while((!backgroundCalculator->isFinished()));

        cout << "Finished background calulcation" << endl;

        cv::imwrite(QString(argv[2]).toStdString(), backgroundCalculator->currentBackground);
    }
}
