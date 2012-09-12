#include "MoG.h"
#include <fstream>
#include <cstdlib>

#include <highgui.h>
#include <limits>

cv::Mat MoG::stauffer_grimson(cv::Mat img, int k, int nChannels, double alpha, double T) {
	static bool isMoG_init = false;
	static int imageSize = img.cols * img.rows;
    static MoG *mog = NULL;

    // initialization the mixture of Gaussians 
    // accroding to the given set of parameters
	if( !isMoG_init ){
		printf("initializing mixtures of Guassians...\n");
		mog = new MoG(imageSize, nChannels, k, alpha, T);
		isMoG_init = true;
	}

    cv::Mat tmp = img.clone();
    cv::GaussianBlur(tmp, img, cv::Size(3,3), 0);

    // update the mixture of Gaussians
    mog->updateMoG( &img );

    tmp = img.clone();
    cv::medianBlur(tmp, img, 3);

    return img;
}

MoG::MoG(int size, int nChannels, int k, double alpha, double T){
    this->k = k;
    this->nChannels = nChannels;    // use single channel as the default
    this->size = size * nChannels * k;

    init(alpha, T);
}

MoG::MoG(int cols, int rows, int nChannels, int k, double alpha, double T){
    MoG(cols*rows, nChannels, k, alpha, T);
}

MoG::~MoG(){
    delete dists;
}

void MoG::init(double alpha, double T){

    // init parameters for Stauffer-Grimson's bgsub algo.
    T_PORTION = T;

    // init all of Gaussians with the given alpha value
    dists = new GaussianDist[size];
    for(int i=0; i<size; i++){
        dists[i].init( 0, 1, alpha );   // mean, numElements, alpha
    }
}

void MoG::setBGFrame( cv::Mat frame ) {
    int width = frame.cols;
    int height = frame.rows;
    cv::Mat3b image = frame;    // 3channels TODO
    for( int y=0; y<height; y++) {
        for( int x=0; x<width; x++ ) 
        {
            // get pixel values (3-channels TODO)
            cv::Point point(x, y);
            const cv::Vec3b& bgr = image(point);

            // find a match from the MoG
            int idx = ( x + (width*y) ) * nChannels * k;    // first idx for the pixel
            for( int i=0; i<nChannels; i++) {
                // for each channels, set the first gaussian's mean as the pixel value 
                dists[idx+i].setMean( bgr[i] );
                dists[idx+i].setVariance( DEFAULT_VAR );
            }
        }
    }
}

/**
 * apply a image information to the mixture of Gaussians
 */
void MoG::updateMoG( cv::Mat *frame ){
    // for each of pixel values, try to match it with the MOG of the pixel
    // if matched or not, updates the MOG accordingly.

    // loop per pixels
    //      get the pixel value
    //      try to match it with the corresponding MOG
    //      (updating MOG would be occured by gaussian obj itself)
    //      get matched information? (to decide FG/BG)

    int width = frame->cols;
    int height = frame->rows;
    cv::Mat3b image = *frame;    // 3channels TODO
    
    int no_match_count = 0;
    int match_count = 0;

    for( int y=0; y<height; y++) {
        for( int x=0; x<width; x++ ) {
            // get pixel values (3-channels TODO)
            cv::Point point(x, y);
            cv::Vec3b& bgr = image(point);
            bgr[0] = 0; // remove blue channel info

            // find a match from the MoG
            int idx = ( x + (width*y) ) * nChannels * k;    // first idx for the pixel

            bool match_allCh = false;
            int matched_k_idx = -1;
            for( int i=0; i<k; i++) {
                // for each channels, find a match 
                int count = 0;
                for( int j=0; j<nChannels; j++) {

                    int idx_k = idx + i*nChannels + j;  // idx for k-th gaussian
                    bool isMatched = dists[idx_k].matching( bgr[j] );    // find a match

                    if( isMatched ) {
                        match_count++;  // DEBUG
                        count++;
                        continue;
                    } else {
                        no_match_count++;   // DEBUG
                        break;
                    }
                }
                if( count == nChannels ){
                    match_allCh = true;     // found a match
                    matched_k_idx = i;
                    break;
                }
            }
            
            // update distributions
            double portion = 0.0;
            for(int i=0; i<k; i++){
                portion = dists[idx+i*nChannels].getWeight();
                for( int j=0; j<nChannels; j++ ){
                    int idx_k = idx + i*nChannels + j;  // idx for k-th gaussian
                    if( i == matched_k_idx ){
                        dists[idx_k].update(true, bgr[j]);
                        // set background pixel as zero
                        if(portion > T_PORTION)
                            for( int j=0; j<nChannels; j++)
                                frame->at<cv::Vec3b>(y, x)[j] = 0;  
                        else
                            for( int j=0; j<nChannels; j++)
                                frame->at<cv::Vec3b>(y, x)[j] = 255;  
                    }
                    else{
                        dists[idx_k].update(false, bgr[j]);
                    }
                }
            }

            // sort the mixtures of Gaussians
            for( int i=0; i<k; i++) {
                for( int h=i; h<k; h++ ){
                    int idx_i = idx + i*nChannels;  // idx for i-th gaussian
                    int idx_h = idx + h*nChannels;  // idx for h-th gaussian
                    double w_i = dists[idx_i].getWeight();
                    double w_h = dists[idx_h].getWeight();
                    if( w_i < w_h){
                        // swap
                        for( int j=0; j<nChannels; j++){
                            GaussianDist tmp = dists[idx_i + j];
                            dists[idx_i + j] = dists[idx_h + j];
                            dists[idx_h + j] = tmp;
                        }
                    }
                }
            }

            // process when there is no match
            //  If none of the K distributions match the current
            //  pixel value, the least probable distribution is replaced
            //  with a distribution with the current value as its mean
            //  value, an initially high variance, and low prior weight.
            if( !match_allCh ) {
                // find the least probable distribution
                int idx_least_weight = idx + (k-1)*nChannels;  // idx for the last gaussian

                // replace the least probable dist with new distribution
                for( int j=0; j<nChannels; j++ ){
                    dists[idx_least_weight + j].replace( bgr[j] );
                    // set foreground pixels as white
                    frame->at<cv::Vec3b>(y,x)[j] = 255;
                }
            }

            // process background pixels 
            // the first T distribution will be treated as background dists
            /*
            for( int i=0; i<k; i++ ){
                int idx_k = idx + i*nChannels;  // idx for k-th gaussian
                double w = dists[idx_k].getWeight();
                portion += w;
                printf("====== portion: %f\n", portion);

                if( portion >= T_PORTION ) break;

                if( match_weight > 0 && w == match_weight )
                    for( int j=0; j<nChannels; j++)
                        frame->at<cv::Vec3b>(y, x)[j] = 0;  //DEBUG

            }
            */
        }
    }

    // DEBUG
#ifdef DEBUG
    printf("match count: \t\t%d\n", match_count);
    printf("no match count: \t%d\n", no_match_count);
    printf("pixel 0's (b) value: %d\n", frame->at<cv::Vec3b>(0,0)[0]);
    for(int i=0; i<k; i++){
        int idx_i = i*nChannels;  // idx for i-th gaussian
        printf("pixel 0's variance (b) [k=%d]: %f\t weight:%f\tmean:%f\n", i, 
                dists[idx_i].getVariance(), 
                dists[idx_i].getWeight(),
                dists[idx_i].getMean());
    }
#endif
}
