/*
 * Twiddler.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: cpierce
 */
#include "Twiddler.h"
#include <iostream>
#include <fstream>
#include <cmath>

Twiddler::Twiddler(double tolerance, vector<PID> pidControllers, vector<double> dts, string outfileName) {
	this->totalCte = 0;
	this->countCte = 0;
	this->maxCte = 0;

	this->tolerance = tolerance;
	this->pidControllers = pidControllers;

	if(dts.size() != pidControllers.size() * 3) {
		std::cerr << "Wrong number of dts, expected " << pidControllers.size() * 3 << ", got " << dts.size() << std::endl;
		throw "Wrong number of dts";
	}
	this->dts = dts;

	this->outfileName = outfileName;
}

Twiddler::~Twiddler(){}


void Twiddler::updateError(double cte) {
	this->totalCte += fabs(cte);
	this->countCte += 1;

	if(this->maxCte < fabs(cte)) {
		this->maxCte = fabs(cte);
	}
}

void Twiddler::twiddle() {
	ofstream outfile;
	outfile.open(this->outfileName, ios::app);

	if(dtSum() < tolerance) {
		outfile << "tolerance met" << endl;
		outfile << "Max Error: " << this->maxCte << endl;
		outfile << "Tot Error: " << this->totalCte << endl;
		outfile << "Avg Error: " << this->totalCte / (double)this->countCte << endl;
		for(unsigned int i = 0; i < pidControllers.size(); ++i) {
			outfile << "PID " << (i + 1) << endl;
			outfile << "    Kp = " << pidControllers[i].Kp << endl;
			outfile << "    Ki = " << pidControllers[i].Ki << endl;
			outfile << "    Kd = " << pidControllers[i].Kd << endl;
		}
		return;
	}
	else {
		outfile << "Max Error: " << this->maxCte << endl;
		outfile << "Tot Error: " << this->totalCte << endl;
		outfile << "Avg Error: " << this->totalCte / (double)this->countCte << endl;
		for(unsigned int i = 0; i < pidControllers.size(); ++i) {
			outfile << "PID " << (i + 1) << endl;
			outfile << "    Kp = " << pidControllers[i].Kp << endl;
			outfile << "    Ki = " << pidControllers[i].Ki << endl;
			outfile << "    Kd = " << pidControllers[i].Kd << endl;
		}
	}


	PID &pidController = pidControllers[curr_pid_idx];
	double dt = dts[dt_idx];

	switch(curr_twiddle_state) {
		case 0: // first time through... baseline
			best_error = totalCte;
			pidController = pidControllers[curr_pid_idx];
			dt = dts[dt_idx];
			switch(curr_error_idx){
				case 0:
					pidController.Kp += dt;
					break;
				case 1:
					pidController.Ki += dt;
					break;
				case 2:
					pidController.Kd += dt;
					break;
			}
			curr_twiddle_state = 1;
			break;

		case 1: // test result "add" test
			outfile << "Best Error:" << this->best_error << endl;

			if(totalCte < best_error) {
				outfile << "Should be keeping this setting: " << curr_error_idx << endl;

				best_error = totalCte;
				dts[dt_idx] *= 1.1;

				nextCoefficient();
				pidController = pidControllers[curr_pid_idx];
				dt = dts[dt_idx];
				switch(curr_error_idx){
					case 0:
						pidController.Kp += dt;
						break;
					case 1:
						pidController.Ki += dt;
						break;
					case 2:
						pidController.Kd += dt;
						break;
				}

				curr_twiddle_state = 1;
			}
			else {

				switch(curr_error_idx){
					case 0:
						pidController.Kp -=  2 * dt;
						break;
					case 1:
						pidController.Ki -=  2 * dt;
						break;
					case 2:
						pidController.Kd -=  2 * dt;
						break;
				}

				curr_twiddle_state = 2;
			}
			break;

		case 2: // test result "subtract" test

			outfile << "Best Error:" << this->best_error << endl;

			if(totalCte < best_error) {
				outfile << "Should be keeping this setting: " << curr_error_idx << endl;
				best_error = totalCte;
				dts[dt_idx] *= 1.1;
			}
			else {
				switch(curr_error_idx){
					case 0:
						pidController.Kp +=  dt;
						break;
					case 1:
						pidController.Ki +=  dt;
						break;
					case 2:
						pidController.Kd +=  dt;
						break;
				}
				dts[dt_idx] *= 0.9;
			}

			nextCoefficient();
			pidController = pidControllers[curr_pid_idx];
			dt = dts[dt_idx];
			outfile << "Moving on to the next: " << curr_error_idx << endl;
			switch(curr_error_idx){
				case 0:
					pidController.Kp += dt;
					break;
				case 1:
					pidController.Ki += dt;
					break;
				case 2:
					pidController.Kd += dt;
					break;
			}

			curr_twiddle_state = 1;
			break;
	}

	outfile.close();

	this->totalCte = 0;
	this->countCte = 0;
	this->maxCte = 0;
	return;
}

void Twiddler::nextCoefficient() {
	curr_error_idx += 1;
	if(curr_error_idx >= 3) {
		curr_error_idx = 0;
		curr_pid_idx += 1;
		if(curr_pid_idx >= pidControllers.size()) {
			curr_pid_idx = 0;
		}
	}
	dt_idx = (curr_pid_idx * 3) + curr_error_idx;

}
double Twiddler::dtSum() {
	double sum = 0.0;
	for(unsigned int i=0; i < dts.size(); ++i) {
		sum += dts[i];
	}
	return sum;
}
