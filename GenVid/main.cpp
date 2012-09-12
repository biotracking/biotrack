#include <QtGui/QApplication>
#include "genvid.h"
/***

Copyright 2012 Andrew Quitmeyer and Georgia Tech's Multi Agent Robotics and Systems Lab www.bio-tracking.org

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

/***

  Todo Genvid
  -UI select multiple encoding types (not just DIVX)



  ***/




int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    qDebug()<<"argc="<<argc;
    QStringList aList = a.arguments();
    GenVid w;
    bool ok;

    if(argc==1){
        w.show();
    }else{
        for(int i=0; i<argc;i++){
            qDebug()<<"Argument i:"<<argv[i];
        }

        w.videopath = argv[1];

        w.btfpath = argv[2];

        w.savepath =argv[3];

        // decode arguments
        if(aList.contains("-box")){
            w.boxOn = true;
            if ((aList.at(aList.indexOf(QString("-box"))+1) != "-tr") ||
                    (aList.at(aList.indexOf(QString("-box"))+1) != "-id") ||
                    (aList.at(aList.indexOf(QString("-box"))+1) != "-cir") ||
                    (aList.at(aList.indexOf(QString("-box"))+1) != "-ar")){

                qDebug()<<aList.at(aList.indexOf(QString("-box"))+1).toInt(&ok)<<"width is an integer:"<<ok;

                if(ok){
                    w.rparam.x = aList.at(aList.indexOf(QString("-box"))+1).toInt();
                    ok=false;
                }
            }

            if ((aList.at(aList.indexOf(QString("-box"))+2) != "-tr") ||
                    (aList.at(aList.indexOf(QString("-box"))+2) != "-id") ||
                    (aList.at(aList.indexOf(QString("-box"))+2) != "-cir") ||
                    (aList.at(aList.indexOf(QString("-box"))+2) != "-ar")){

                qDebug()<<aList.at(aList.indexOf(QString("-box"))+2).toInt(&ok)<<"height is an integer:"<<ok;

                if(ok){
                    w.rparam.y = aList.at(aList.indexOf(QString("-box"))+2).toInt();
                    ok=false;
                }
            }

        }

        if(aList.contains("-s")){
            w.fontSize = 1;
        }

        if(aList.contains("-l")){
            w.fontSize =2;
        }
        if(aList.contains("-id")){
            w.idOn = true;
        }
        if(aList.contains("-xy")){
            w.xyOn = true;
        }
        if(aList.contains("-ang")){
            w.angleOn = true;
        }


//Circle Features
        if(aList.contains("-cir")){
            w.cirOn = true;
        }

        if(aList.contains("-rad")){
            w.cirOn = true;
            if (
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-id") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-xy") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-ang") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-s") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-l") ||

                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-cir") ||
//                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-rad") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-cs") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-ccol") ||

                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-tr") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-tsize") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-cs") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-tcol") ||

                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-ar") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-asize") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-ars") ||
                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-acol") ||

                    (aList.at(aList.indexOf(QString("-rad"))+1) != "-box")
                    ){

                aList.at(aList.indexOf(QString("-rad"))+1).toInt(&ok);

                if(ok){
                    w.rad = aList.at(aList.indexOf(QString("-rad"))+1).toInt();
                    ok=false;
                }
            }
        }
        if(aList.contains("-cs")){
            w.cirOn = true;
            if (
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-id") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-xy") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-ang") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-s") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-l") ||

                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-cir") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-rad") ||
//                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-cs") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-ccol") ||

                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-tr") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-tsize") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-trs") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-tcol") ||

                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-ar") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-asize") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-ars") ||
                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-acol") ||

                    (aList.at(aList.indexOf(QString("-cs"))+1) != "-box")
                    ){

                aList.at(aList.indexOf(QString("-cs"))+1).toInt(&ok);

                if(ok){
                    w.cirStroke = aList.at(aList.indexOf(QString("-cs"))+1).toInt();
                    ok=false;
                }
            }
        }
        if(aList.contains("-ccol")){
            w.cirOn = true;
            w.cirU = true;
        }


//Trail Features
        if(aList.contains("-tr")){
            w.trailOn = true;
        }

        if(aList.contains("-tsize")){
            w.trailOn = true;
            if (
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-id") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-xy") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-ang") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-s") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-l") ||

                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-cir") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-rad") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-cs") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-ccol") ||

                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-tr") ||
//                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-tsize") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-trs") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-tcol") ||

                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-ar") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-asize") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-ars") ||
                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-acol") ||

                    (aList.at(aList.indexOf(QString("-tsize"))+1) != "-box")
                    ){

                aList.at(aList.indexOf(QString("-tsize"))+1).toInt(&ok);

                if(ok){
                    w.trailSize = aList.at(aList.indexOf(QString("-tsize"))+1).toInt();
                    ok=false;
                }
            }
        }
        if(aList.contains("-trs")){
            w.trailOn = true;
            if (
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-id") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-xy") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-ang") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-s") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-l") ||

                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-cir") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-rad") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-cs") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-ccol") ||

                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-tr") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-tsize") ||
//                   (aList.at(aList.indexOf(QString("-trs"))+1) != "-trs") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-tcol") ||

                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-ar") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-asize") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-ars") ||
                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-acol") ||

                    (aList.at(aList.indexOf(QString("-trs"))+1) != "-box")
                    ){

                aList.at(aList.indexOf(QString("-trs"))+1).toInt(&ok);

                if(ok){
                    w.trailStroke = aList.at(aList.indexOf(QString("-trs"))+1).toInt();
                    ok=false;
                }
            }
        }
        if(aList.contains("-tcol")){
            w.trailOn = true;
            w.trailU = true;
        }


        //Arrow Features
                if(aList.contains("-ar")){
                    w.arrowOn = true;
                }

                if(aList.contains("-asize")){
                    w.arrowOn = true;
                    if (
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-id") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-xy") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-ang") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-s") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-l") ||

                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-cir") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-rad") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-cs") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-ccol") ||

                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-tr") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-tsize") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-cs") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-tcol") ||

                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-ar") ||
        //                    (aList.at(aList.indexOf(QString("-asize"))+1) != "-asize") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-ars") ||
                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-acol") ||

                            (aList.at(aList.indexOf(QString("-asize"))+1) != "-box")
                            ){

                        aList.at(aList.indexOf(QString("-asize"))+1).toInt(&ok);

                        if(ok){
                            w.arrowSize = aList.at(aList.indexOf(QString("-asize"))+1).toInt();
                            ok=false;
                        }
                    }
                }
                if(aList.contains("-ars")){
                    w.arrowOn = true;
                    if (
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-id") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-xy") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-ang") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-s") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-l") ||

                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-cir") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-rad") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-cs") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-ccol") ||

                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-tr") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-tsize") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-trs") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-tcol") ||

                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-ar") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-asize") ||
        //                    (aList.at(aList.indexOf(QString("-ars"))+1) != "-ars") ||
                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-acol") ||

                            (aList.at(aList.indexOf(QString("-ars"))+1) != "-box")
                            ){

                        aList.at(aList.indexOf(QString("-ars"))+1).toInt(&ok);

                        if(ok){
                            w.arrowStroke = aList.at(aList.indexOf(QString("-ars"))+1).toInt();
                            ok=false;
                        }
                    }
                }
                if(aList.contains("-ccol")){
                    w.arrowOn = true;
                    w.arrU = true;
                }


            w.cmd =true;
            w.on_actionLoad_BTF_triggered();

            if(w.isPlaying){
                w.on_exportButton_clicked();
            }


        }

    return a.exec();
}
