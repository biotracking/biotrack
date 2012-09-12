#ifndef ABOSPOOL_H
#define ABOSPOOL_H

#include <vector>
#include <sys/time.h> 	// timestamp?
#include <cv.h>
#include <highgui.h>	// opencv's gui. it may be removed later. TODO
#include <qmutex.h>		// mutex for... reference counter?
#include <qthread.h>

#include "poolframe.h"

// default values of parameters
#define 	MAX_POOL_SIZE			50			// unit: images (50 images)
#define		DEFAULT_TEMP_LENGTH		5			// unit: seconds (5 sec.)

/** 
 * This class contains image frames and helps transmitting frames between modules.
 *
 * It is designed to provide simple way to avoid the complexity of data structure
 * and memory manipulation between threads. Every modules on ABOS architecture 
 * should be leveraged by this class when they try to send or get frames. Each 
 * module can grab any frames available from one or more pools and store their 
 * result frames to for later use by other threads (or by itself also).
 */

class AbosPool : public QThread{
	public:
		AbosPool();
		~AbosPool();

		// get a frame
		PoolFrame* getRecentFrame();
		PoolFrame* getOldestFrame();
		PoolFrame* getFrameByNumber(unsigned long long number);

		// set images
		void storeImage(cv::Mat img);
		void storeImage(cv::Mat img, unsigned long long number);

		// set frames
		void storeFrame(PoolFrame* frame);

		// release a frame 
		void releaseFrame(PoolFrame* frame);

		// clear pool
		void clear();

		// periodic checkup for releasing frames
		void run();
		void stop(){ this->stopped = true; }

		// setters and getters
		int getMaxSize(){ return max_size; }
		int getSize(){ return frame_vector.size(); }
		int getTemporalLength(){ return temporal_length; }
		void setMaxSize(int size){ this->max_size = size; }
		void setTemporalLength(int length){ this->temporal_length = length; }

	private:
		// parameters
		int max_size;
		int temporal_length;

		// storage for frames
		std::vector<PoolFrame*> frame_vector;
		unsigned long long frame_number;

		// mutex
		QMutex mutex_refcount;
		bool stopped;

};

#endif // ABOSPOOL_H
