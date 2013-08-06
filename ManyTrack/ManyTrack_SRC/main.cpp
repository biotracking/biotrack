#include <QtGui/QApplication>
#include "Manytrack.h"
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


/**


  TODO- figure out 0,0 error //POSSIBLE SOLUTIONS - unlimted search range, shiftedness of pixels in bad code?

  **/

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    Manytrack w;
    if(argc<=1){
        w.show();
        return a.exec();
    }else{
        w.loadSettings(QString(argv[1]));
        w.startTracking();
        cout << "Started tacking" << endl;
        while(!w.isTrackingCompleted())
            w.track();
        cout << "Finished tracking" << endl;
        w.saveBTF();
        cout << "Saved" << endl;
        return 0;
    }

    //Shows Path for current environment
   // cout<<getenv("PATH")<<endl;

}




