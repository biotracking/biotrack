#include "abospool.h"
#include <iostream>

/**
 * Default constructor
 */
AbosPool::AbosPool() {
	max_size = MAX_POOL_SIZE;
	temporal_length = DEFAULT_TEMP_LENGTH;
	frame_number = 0;
	std::cout << "pool initialized" << std::endl;

	stopped = false;
	this->start();
}

/**
 * Default destructor
 */
AbosPool::~AbosPool(){
	this->clear();
}

/**
 * Release all frames contained and empty the pool
 * WARNING: this will delete all frames no matter with reference counters.
 */
void AbosPool::clear(){
	// if the storage is not empty, free occupied memories. 
	while( frame_vector.size() > 0 ) {
		PoolFrame *tmp = frame_vector.front();
		frame_vector.erase( frame_vector.begin() );

		delete tmp;
	}

	frame_vector.clear();
}


/**
 * Pools acts as like a thread. 
 * It sleeps most of time, and wake up by periodical time distances 
 * to search and release frames whose ref_counter is 0 which means 
 * it is not being used by any modules.
 */
void AbosPool::run() {

	do {	// thread main loop

		QThread::sleep( temporal_length ); // sleep (in 'temporal_length' seconds)

		// wake up to check if the pool has been exceeded.
		int size = frame_vector.size();
		//std::cout << "pool size: " << size << std::endl;

		if( size > max_size ) {
			// here, we have exceeded frames. 
			// release a ref_count occupied by the pool itself if there are any 'expired' frames
			mutex_refcount.lock();
			std::vector<PoolFrame*>::iterator it;
			int exceed = size - max_size;	// this should be a positive integer
			it = frame_vector.begin();
			for( int i=0; i<exceed; i++ ){
				PoolFrame* frame = *it;
				frame->release();	// decrease the reference counter
				if( frame->isExpired() ){
					it = frame_vector.erase( it ); // erase from the pool
					delete frame;
				}
				else it++;
			}
			mutex_refcount.unlock();
		}

		// delete expired frames to release allocate memory 
		/*
		std::vector<PoolFrame*>::iterator it;
		for( it=frame_vector.begin(); it<frame_vector.end(); ){
			PoolFrame* frame = *it;
			bool expired = frame->isExpired();	
			if( expired ){
				it = frame_vector.erase( it );	// erase from the pool
				delete frame;	// delete from the memory
			}
			else it++;
		}
		*/

	} while (!stopped);		// thread main loop
}

/** 
 * Stores an image to the pool.
 * It copies the given image and store it to the pool. That means, 
 * after storing an image to the pool, you can modify or delete
 * the original image without ruining any images in the pool. 
 *
 * The frame number will be assigned automatically.
 */
void AbosPool::storeImage(cv::Mat img){

	// by creating new object of PoolFrame, 
	// the given image will be copied.
	// reference counter will be initialized as 1 at first
	// (meaning reserved by pool itself. will be released after TEMP_LENGTH)
	PoolFrame *frame = new PoolFrame(img, frame_number);
	frame_vector.push_back(frame);

	frame_number++; 	// frame_number increasing
}

/** 
 * Stores an image to the pool.
 * It copies the given image and store it to the pool. That means, 
 * after storing an image to the pool, you can modify or delete
 * the original image without ruining any images in the pool. 
 *
 * Use this if you want to specify the specific frame number.
 */
void AbosPool::storeImage(cv::Mat img, unsigned long long number){

	// by creating new object of PoolFrame, 
	// the given image will be copied.
	// reference counter will be initialized as 1 at first
	// (meaning reserved by pool itself. will be released after TEMP_LENGTH)
	PoolFrame *frame = new PoolFrame(img, number);
	frame_vector.push_back(frame);
}

/**
 * Stores a frame to the pool
 * It clones the given frame and store to the pool.
 * By storing frame, you can maintain frame_number across pools
 */
void AbosPool::storeFrame(PoolFrame* frame) {
	const cv::Mat* image = frame->getImage();
	cv::Mat new_image = image->clone();

	PoolFrame *new_frame = new PoolFrame( new_image, frame->getFrameNumber() );
	frame_vector.push_back(new_frame);
}


/**
 * Returns the most recent image from the pool.
 * Note: The returned Mat will be IMMUTABLE to maintain data intgrity 
 *       of the pool. If you want to modify the image, you first need 
 *       to make a copy of it manually.
 */
PoolFrame* AbosPool::getRecentFrame() {

	// assumption: the most recent frame is the last entry of the vector
	//             since we pushes frames back everytime.
	if( frame_vector.size() == 0 )
		return NULL;

	mutex_refcount.lock();

	PoolFrame *frame = frame_vector.back();
	frame->allocate();	// increase reference counter

	mutex_refcount.unlock();

	return frame;
}

/**
 * Returns the oldest image from the pool.
 * Note: The returned Mat will be IMMUTABLE to maintain data intgrity 
 *       of the pool. If you want to modify the image, you first need 
 *       to make a copy of it manually.
 */
PoolFrame* AbosPool::getOldestFrame() {
	
	// assumption: the oldest frame is the first entry of the vector
	//             since we push_backed frames.
	if( frame_vector.size() == 0 )
		return NULL;

	mutex_refcount.lock();

	PoolFrame* frame = frame_vector.front();
	frame->allocate();	// increase reference counter

	mutex_refcount.unlock();

	return frame;
}

/**
 * Returns an image with the given frame number.
 * If there is no matching entry, it will return NULL pointer.
 * Note: The returned Mat will be IMMUTABLE to maintain data intgrity 
 *       of the pool. If you want to modify the image, you first need 
 *       to make a copy of it manually.
 */
PoolFrame* AbosPool::getFrameByNumber(unsigned long long number){

	if( frame_vector.size() == 0 ){
		return NULL;
	}

	PoolFrame *ret = NULL;

	mutex_refcount.lock();
	// searching
	std::vector<PoolFrame*>::iterator it;
	for( it=frame_vector.begin(); it<frame_vector.end(); it++ ){
		PoolFrame *frame = *it;
		if( frame->getFrameNumber() == number ){
			// returning
			frame->allocate();	// increase reference counter
			ret = frame;
		}
	}
	mutex_refcount.unlock();

	return ret;
}

/**
 * Release the given frame which means decreasing the referenece counter
 * of the frame.
 */
void AbosPool::releaseFrame(PoolFrame* frame){

	if( frame_vector.size() == 0 )
		return ;

	mutex_refcount.lock();

	frame->release();		// decrease the ref_count

	// if the frame has been expired, delete the expired frame
	if( frame->isExpired() ){
		// searching
		std::vector<PoolFrame*>::iterator it;
		for( it=frame_vector.begin(); it<frame_vector.end(); ){
			PoolFrame *current = *it;
			if( current->getFrameNumber() == frame->getFrameNumber() ){
				// delete the expired frame
				it = frame_vector.erase( it );
				delete current;
				break;
			}
			else it++;
		}
	}

	mutex_refcount.unlock();
}






