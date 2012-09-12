#include "poolframe.h"
#include <cstdio>
#include <cassert>

PoolFrame::PoolFrame(cv::Mat img, unsigned long long frame_number){
	image = img.clone();
	this->frame_number = frame_number;
	
	mutex_ref_count.lock();
	ref_count = 1;
	mutex_ref_count.unlock();
}
PoolFrame::~PoolFrame(){
}

void PoolFrame::release(){
	mutex_ref_count.lock();

	assert( (ref_count-1) >= 0 );
	ref_count--;

	mutex_ref_count.unlock();
}

void PoolFrame::allocate(){
	mutex_ref_count.lock();

	assert( (ref_count) >= 0 );
	ref_count++;

	mutex_ref_count.unlock();
}

/**
 * return true if reference counter is 0
 * return false otherwise.
 */
bool PoolFrame::isExpired(){
	assert( (ref_count) >= 0 );
	if( ref_count == 0)
		return true;
	else return false;
}
