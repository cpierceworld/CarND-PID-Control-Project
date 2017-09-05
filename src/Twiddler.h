/*
 * Twiddler.h
 *
 *  Created on: Aug 27, 2017
 *      Author: cpierce
 */

#ifndef SRC_TWIDDLER_H_
#define SRC_TWIDDLER_H_

#include "PID.h"
#include <vector>
#include <string>
#include <limits>

using namespace std;

class Twiddler {
private:
	double totalCte;
	double maxCte;
	double countCte;

	double tolerance = 0.0;
	unsigned int curr_pid_idx = 0;
	unsigned int curr_error_idx = 0;
	unsigned int curr_twiddle_state = 0;
	unsigned int dt_idx = 0;

	bool best_error_initialized = false;
	double best_error = std::numeric_limits<double>::max();

	vector<PID> pidControllers;
	vector<double> dts;

	string outfileName;

	double dtSum();

	void nextCoefficient();

public:
	Twiddler(double tolerance, vector<PID> pidControllers, vector<double> dts, string outfileName);
	virtual ~Twiddler();

	void updateError(double cte);
	void twiddle();

};

#endif /* SRC_TWIDDLER_H_ */
