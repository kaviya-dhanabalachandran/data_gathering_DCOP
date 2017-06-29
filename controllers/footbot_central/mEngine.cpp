/*
 * mEngine.cpp
 *
 *  Created on: 13/10/2014
 *      Author: roberto
 */

#include "mEngine.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

using namespace std;

mEngine::mEngine() {
	// TODO Auto-generated constructor stub

}

vector<double> mEngine::runEgine(double * un_positions, int unNumber, double * an_positions, int anNumber, double * vit_4_last, double horizont, double e_weight, double time, double debug) {

	mxArray *orig_UN = NULL, *orig_AN, *result, * vit4last;

	mxArray * H, *edge_w, *t, *dbg;

	std::vector<double> AN;

	Engine *ep;
	if (!(ep = engOpen("matlab -nosplash"))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	} else {
		fprintf(stderr, "\n MATLAB engine OK!\n");
	}

	// UN locations
	orig_UN = mxCreateDoubleMatrix(1, unNumber * 2, mxREAL);
	//mxSetData(orig_UN,un_positions);
	memcpy(mxGetPr(orig_UN), un_positions, unNumber * 2 * sizeof(double));

	engPutVariable(ep, "orig_UN", orig_UN);

	//AN locations
	orig_AN = mxCreateDoubleMatrix(1, anNumber * 2, mxREAL);
	//mxSetData(orig_AN,an_positions);
	memcpy(mxGetPr(orig_AN), an_positions, anNumber * 2 * sizeof(double));

	engPutVariable(ep, "orig_AN", orig_AN);

	//4 last UN locations
	vit4last = mxCreateDoubleMatrix(1, unNumber * 2 * 4, mxREAL);
	//mxSetData(orig_AN,an_positions);
	memcpy(mxGetPr(vit4last), vit_4_last, unNumber * 2 * 4 * sizeof(double));

	//4 last UN locations
	engPutVariable(ep, "vit_4_last", vit4last);


//	for (unsigned i = 0; i < unNumber * 2 * 4; i++) {
//		printf("last : %f\n",vit_4_last[i]);
//	}

	// H
	H = mxCreateDoubleScalar(horizont);
	engPutVariable(ep, "H", H);

	// edge_w
	edge_w = mxCreateDoubleScalar(e_weight);
	engPutVariable(ep, "edge_w", edge_w);

	// t
	t = mxCreateDoubleScalar(time);
	engPutVariable(ep, "t", t);

	// dbg
	dbg = mxCreateDoubleScalar(debug);
	engPutVariable(ep, "dbg", dbg);

	engEvalString(ep, "addpath('/home/roberto/RMAGAN/Dropbox/THESIS/ESTANCIA/MATLAB/CODE/')");
	engEvalString(ep, "addpath('/home/roberto/RMAGAN/Dropbox/THESIS/ESTANCIA/MATLAB/CODE/gaimc/')");
	engEvalString(ep, "[AN_t1_polar, AN_t1, APs] = entry_point(orig_UN,orig_AN, vit_4_last,H,edge_w,t,dbg)");

	//engGetVariable(ep, "output");

	if ((result = engGetVariable(ep, "AN_t1")) == NULL)
		printf("Oops! You didn't create a variable output.\n\n");
	else {
		//printf();
	}

	AN.assign(mxGetPr(result), mxGetPr(result) + mxGetNumberOfElements(result));

	printf("AN %i",AN.size());
	//cout << AN.size();

	for (unsigned i = 0; i < AN.size(); i++) {
		cout << ' ' << AN.at(i) << '\n';
	}

	mxDestroyArray(result);
	engClose(ep);

	return AN;

}

vector<double> mEngine::runEgine() {

	mxArray *orig_UN = NULL, *orig_AN, *result, * vit4last;

	mxArray * H, *edge_w, *t, *dbg;

	std::vector<double> AN;

	Engine *ep;
	if (!(ep = engOpen("matlab -nosplash"))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	} else {
		fprintf(stderr, "\n MATLAB engine OK!\n");
	}


	// UN locations
	orig_UN = mxCreateDoubleMatrix(1, unNumber * 2, mxREAL);
	//mxSetData(orig_UN,un_positions);
	memcpy(mxGetPr(orig_UN), unPositions, unNumber * 2 * sizeof(double));

	engPutVariable(ep, "orig_UN", orig_UN);

	printf("1\n");

	//AN locations
	orig_AN = mxCreateDoubleMatrix(1, anNumber * 2, mxREAL);
	//mxSetData(orig_AN,an_positions);
	printf("2\n");

	memcpy(mxGetPr(orig_AN), anPositions, anNumber * 2 * sizeof(double));
	printf("3\n");



	engPutVariable(ep, "orig_AN", orig_AN);

	//4 last UN locations
	vit4last = mxCreateDoubleMatrix(1, unNumber * 2 * 4, mxREAL);
	//mxSetData(orig_AN,an_positions);
	memcpy(mxGetPr(vit4last), vit_4_last, unNumber * 2 * 4 * sizeof(double));

	//4 last UN locations
	engPutVariable(ep, "vit_4_last", vit4last);


//	for (unsigned i = 0; i < unNumber * 2 * 4; i++) {
//		printf("last : %f\n",vit_4_last[i]);
//	}

	// H
	H = mxCreateDoubleScalar(horizont);
	engPutVariable(ep, "H", H);

	// edge_w
	edge_w = mxCreateDoubleScalar(e_weight);
	engPutVariable(ep, "edge_w", edge_w);

	// t
	t = mxCreateDoubleScalar(time);
	engPutVariable(ep, "t", t);

	// dbg
	dbg = mxCreateDoubleScalar(debug);
	engPutVariable(ep, "dbg", dbg);

	engEvalString(ep, "addpath('/home/roberto/RMAGAN/Dropbox/THESIS/ESTANCIA/MATLAB/CODE/')");
	engEvalString(ep, "addpath('/home/roberto/RMAGAN/Dropbox/THESIS/ESTANCIA/MATLAB/CODE/gaimc/')");
	engEvalString(ep, "[AN_t1_polar, AN_t1, APs] = entry_point(orig_UN,orig_AN, vit_4_last,H,edge_w,t,dbg)");

	//engGetVariable(ep, "output");

	if ((result = engGetVariable(ep, "AN_t1")) == NULL)
		printf("Oops! You didn't create a variable output.\n\n");
	else {
		//printf();
	}

	AN.assign(mxGetPr(result), mxGetPr(result) + mxGetNumberOfElements(result));

//	printf("AN %i",AN.size());
//	cout << AN.size();

	for (unsigned i = 0; i < AN.size(); i++) {
		cout << ' ' << AN.at(i) << '\n';
	}

	mxDestroyArray(result);
	engClose(ep);

	return AN;

}

mEngine::~mEngine() {
	// TODO Auto-generated destructor stub
}

