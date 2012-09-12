#include "GaussianDist.h"
#include <iostream>
#include <fstream>
#include <cstdlib>


GaussianDist::GaussianDist(){
	GaussianDist(0,0,DEFAULT_ALPHA);
}

GaussianDist::GaussianDist(double mean, int numElements, double alpha){
    init(mean, numElements, alpha);
}

void GaussianDist::init(double mean, int numElements, double alpha){
	this->mean = mean;
	this->variance = DEFAULT_VAR;
    this->numElements = numElements;

	weight = DEFAULT_WEIGHT;		// initial value of the prior weight
	//alpha = DEFAULT_ALPHA;		// initial value of the learning rate
    this->alpha = alpha;
}

/**
 * Test matching with given value
 * A match is definced as a pixel value within 2.5 standard deviation
 *
 * To update the distribution, one should call update() explicitly.
 *
 */
bool GaussianDist::matching(double value){
	bool ret = false;

	double deviation = std::abs(value - mean);
	double std_dev = sqrt(variance);

	if( deviation / std_dev < MATCH_RANGE ) ret = true;
	else ret = false;

	return ret;
}

/**
 * Update the Gaussian distribution according to whether matched or not.
 * If a match is found, then add the value as an element to the distritbution.
 * otherwise, the prior weight would be decreased with the learning rate.
 */
void GaussianDist::update(bool matched, double value){
	if( matched ){
		addElement(value);  // update mean and variance
		weight = (1-alpha) * weight + alpha;
	}
	else{
        // mean and variance for unmatched distributions remains same
		weight = (1-alpha) * weight;
	}
}

/**
 * add one element and update the mean and variance accordingly
 */
void GaussianDist::addElement(double value){
	// learning rates 
	double eta = this->gaussianPDF(value);
	double rho = alpha * eta;

	/*
	std::cout << "adding: eta = " << eta << std::endl;
	std::cout << "adding: rho = " << rho << std::endl;
	std::cout << "adding: mean = " << mean << std::endl;
	std::cout << "adding: std.dev = " << sqrt(variance) << std::endl;
	std::cout << "adding: variance = " << variance << std::endl;
	*/

	// add the element
	numElements++;

	// update parameters
	mean = (1-rho) * mean + rho * value;
    if( variance > MIN_VAR )
        variance = (1-rho)*variance + rho * pow(value-mean, 2);
}
/**
 * return the probability based on this distribution's
 * probability density function
 */
double GaussianDist::gaussianPDF(double value){
	double eta = 0;
	double variance2 = 2 * variance;

	eta =  1/sqrt(M_PI * variance2) * exp(-1 * pow(value-mean,2) / variance2);
	return eta;
}

/**
 * replace the distribution with new mean and default variance.
 * the weight will also be initialized.
 */
void GaussianDist::replace(double value) {
    this->mean = value;
    this->variance = DEFAULT_VAR;
    this->numElements = 0;
    //this->alpha = DEFAULT_ALPHA;
    this->weight = DEFAULT_WEIGHT;
}

// test cases
/*
int main(){
	double mean = 5;
	double std_dev = 2;
	double numElem = 5;
	GaussianDist gdist(mean, pow(std_dev,2), numElem);
	double eta = gdist.gaussianPDF(3);
	std::cout << "eta(t-1) = " << gdist.gaussianPDF(3) << std::endl;
	std::cout << "mean(t-1) = " << gdist.getMean() << std::endl;
	std::cout << "variance(t-1) = " << gdist.getVariance() << std::endl;
	std::cout << "weight of the distribution(t-1) = " << gdist.getWeight() << std::endl;
	std::cout << std::endl;
	// keep trying to add 3
	for(int i=0; i<20; i++){
		gdist.matching(3);
		std::cout << "weight of the distribution(t) = " << gdist.getWeight() << std::endl;
	}
	std::cout << "eta(t) = " << gdist.gaussianPDF(3) << std::endl;
	std::cout << "mean(t) = " << gdist.getMean() << std::endl;
	std::cout << "variance(t) = " << gdist.getVariance() << std::endl;
	std::cout << "weight of the distribution(t) = " << gdist.getWeight() << std::endl;
	std::cout << std::endl;

	// keep trying to add 9
	for(int i=0; i<20; i++){
		gdist.matching(9);
		std::cout << "weight of the distribution(t+1) = " << gdist.getWeight() << std::endl;
	}
	std::cout << "eta(t+1) = " << gdist.gaussianPDF(9) << std::endl;
	std::cout << "mean(t+1) = " << gdist.getMean() << std::endl;
	std::cout << "variance(t+1) = " << gdist.getVariance() << std::endl;
	std::cout << "weight of the distribution(t+1) = " << gdist.getWeight() << std::endl;
	std::cout << std::endl;


    std::cout << "Testing speed..." << std::endl;




	return 0;
}
*/
