/*
 * mEngine.h
 *
 *  Created on: 13/10/2014
 *      Author: roberto
 */

#include "engine.h"
#include <vector>

#ifndef MENGINE_H_
#define MENGINE_H_

using namespace std;

class mEngine {
public:
	mEngine();

	vector<double> runEgine(double * un_positions, int unNumber, double * an_positions, int anNumber, double * vit_4_last, double horizont, double e_weight, double time, double debug);
	vector<double> runEgine();

	double * unPositions;
	double * anPositions;
	int anNumber;
	int unNumber;
	double * vit_4_last;
	double horizont;
	double e_weight;
	double time;
	double debug;

	/*Getters & Setters*/

	virtual ~mEngine();

	int getAnNumber() const {
		return anNumber;
	}

	void setAnNumber(int anNumber) {
		this->anNumber = anNumber;
	}

	double * getAnPositions() {
		return anPositions;
	}

	void setAnPositions(double * anPositions) {
		this->anPositions = anPositions;
	}

	double getDebug() {
		return debug;
	}

	void setDebug(double debug) {
		this->debug = debug;
	}

	double getWeight() {
		return e_weight;
	}

	void setWeight(double weight) {
		e_weight = weight;
	}

	double getHorizont() {
		return horizont;
	}

	void setHorizont(double horizont) {
		this->horizont = horizont;
	}

	double getTime() {
		return time;
	}

	void setTime(double time) {
		this->time = time;
	}

	int getUnNumber() {
		return unNumber;
	}

	void setUnNumber(int unNumber) {
		this->unNumber = unNumber;
	}

	double* getUnPositions() {
		return unPositions;
	}

	void setUnPositions(double* unPositions) {
		this->unPositions = unPositions;
	}

	double* getVit4Last() {
		return vit_4_last;
	}

	void setVit4Last(double* vit4Last) {
		vit_4_last = vit4Last;
	}
};

#endif /* MENGINE_H_ */
