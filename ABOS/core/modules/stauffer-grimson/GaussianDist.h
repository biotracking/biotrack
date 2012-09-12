#ifndef GAUSSIANDIST_H
#define GAUSSIANDIST_H

#include <cmath>

#ifndef M_PI	// supporting cross-platform
#define M_PI 3.14159265358979323846
#endif

#define	DEFAULT_WEIGHT  0.1		// initial value of the prior weight
#define DEFAULT_ALPHA   0.01	// initial value of the learning rate
#define DEFAULT_VAR     16      // initial value of variance
#define MIN_VAR         1
#define MATCH_RANGE     2.5     
//#define MATCH_RANGE     5.0     // a match is defined as a pixel value 
                                //  within 2.5 standard deviations
                                

/**
 * This class is implemented a single Gaussian distribution.
 */
class GaussianDist
{
private:
	double mean;
	double variance;
	int numElements;

	double weight;		// the prior weight of the distribution
	double alpha;		// learning rate for the distribution

public:
	// constructors
	GaussianDist();     // initialized as mean=0, noEle=0
	GaussianDist(double mean, int numElements, double alpha);
    void init(double mean, int numElements, double alpha);

	/**
	 * Test matching with given value
	 * A match is definced as a pixel value within 2.5 standard deviation
	 */
	bool matching(double value);

    /**
     * update Gaussians according to matched information
     */
    void update(bool matched, double value);

	/**
	 * add one element and compute the mean and variance again
	 */
	void addElement(double value);

	/**
	 * return the probability based on this distribution's
	 * probability density function
	 */
	double gaussianPDF(double value);
	
	// getters
	double getMean(){ return this->mean; }
	double getVariance(){ return this->variance; }
	double getWeight() { return this->weight; }
	double getAlpha() { return this->alpha; }

	// setters
	void setAlpha(double alpha){ this->alpha = alpha; }		
	void setWeight(double weight){ this->weight = weight; }
    void setMean(double mean) { this->mean = mean; }
    void setVariance(double variance) { this->variance = variance; }
    void replace(double value);

    // operator overloading
    GaussianDist& operator=(const GaussianDist& dist){
        this->mean = dist.mean;
        this->variance = dist.variance;
        this->numElements = dist.numElements;
        this->alpha = dist.alpha;
        this->weight = dist.weight;

        return *this;
    }
};

#endif
