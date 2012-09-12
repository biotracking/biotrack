#ifndef ABOSTHREAD_H
#define ABOSTHREAD_H

#include <qthread.h>
#include <cv.h>
#include <highgui.h>
#include <qmutex.h>

//#include "imagedatapool.h"
#include "abospool.h"

/**
 *  A class that represents a single image processing module as a runnable
 *  QThread.  This is the main entry point for user created subclasses. In
 *  general, a user will inherit from AbosThread and overload the run()
 *  method. Simple modules will set up a loop in the run() method that 
 *  processes one frame at a time by grabbing a PoolFrame from one AbosPool,
 *  processing it, and storing the result in another AbosPool. More complex
 *  modules may take multiple frames as input and produce multiple output
 *  frames.
 */
class AbosThread : public QThread
{
public:
	/**
	 *  Default constructor.  It's a good idea to overload this method with
	 *  sane defaults for your subclasses.
	 */
	AbosThread();

	/**
	 *  Default destructor. It's a good idea to overload this method with
	 *  sane defaults for your subclasses. AbosThread itself has no dynamic
	 *  memory allocation to manage.
	 */
	virtual ~AbosThread();

	/**
	 *  This method is the one you want to overload in subclasses of 
	 *  AbosThread.  Is expected to terminate if stop() is called. Is expected
	 *  to loop at roughly the speed set by setFPS() (or slower).
	 */

    virtual void run() = 0;
	/**
	 *  This method should handle stopping the run() method as well as any
	 *  cleanup.  This method isn't garanteed to be called before the
	 *  destructor.
	 */ 
    virtual void stop();

	virtual void setFPS(int fps) { this->fps = fps; } 
	virtual int getFPS() { return fps; }
	

protected:
    bool stopped;
	int fps;
};



#endif // ABOSTHREAD_H

