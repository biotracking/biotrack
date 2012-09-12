#include "AbosThread.h"

AbosThread::AbosThread(){
	stopped = false;
}

AbosThread::~AbosThread(){

}

/*
void AbosThread::setReadPool(AbosPool *readp){

    readpool = readp;
    set_readpool = true;
}

void AbosThread::setWritePool(AbosPool *writep){
    writepool = writep;
    set_writepool = true;
}

AbosPool* AbosThread::getWritePool(){
    if(set_writepool == true)
        return writepool;
    else
        return NULL;
}


bool AbosThread::isReadPoolSet(){
    return set_readpool;
}
*/

void AbosThread::stop(){
    stopped = true;

}
