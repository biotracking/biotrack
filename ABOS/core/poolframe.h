#ifndef POOLFRAME_H
#define POOLFRAME_H

#include "cv.h"
#include "qmutex.h"

/**
 *  An immutable image data structure.  A PoolFrame is intended to be the
 *  base data structure for images passed between ABOS modules in 
 *  AbosPool 's.  AbosPool 's should be the only objects that create,
 *  destroy, or modify PoolFrame objects. This allows each frame to be shared
 *  among multiple ABOS modules concurrently. 
 */

class PoolFrame {
	//friend class ImageDataPool;
	friend class AbosPool;
	public:

		/**
		 *  Returns the image wrapped by this PoolFrame. Make a .clone() of
		 *  the image if you need to modify it.
		 */
		const cv::Mat* getImage() const { return &image; };
		unsigned long long getFrameNumber() const { return frame_number; };

		// reference counter control

		/**
		 *  Decrement reference counter. ABOS modules should call this when
		 *  they're done with a PoolFrame given to them by an AbosPool
		 */
		void release();
		void allocate();

		// allocation control
		bool isExpired();


	private:
		/**
		 * Constructor. Takes an image and a frame number and creates
		 * the appropriate PoolFrame wrapper. Should only be called by
		 * an AbosPool on adding a new frame to the pool.
		 */
		PoolFrame(cv::Mat img,unsigned long long frame_number);

		/**
		 * Destructor. Handles cleaning up the image using OpenCV's
		 * memory management semantics.
		 */
		~PoolFrame();
		cv::Mat image;
		unsigned long long frame_number;
		QMutex mutex_ref_count;
		int ref_count;
	
};

#endif	// POOLFRAME_H
