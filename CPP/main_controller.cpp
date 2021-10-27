/*
 * main_controller.cpp
 *
 *  Created on: 31 gru 2020
 *      Author: Łukasz Zelek
 */

#include <main_controller.hpp>
#include <vector>
#include <algorithm>
#include <queue>
// #include <iostream> // this header is too large - 210 kb dependecies

void startSystem(){
	// detect walls, 33 Hz - we update x,y , matrix Hloc <- 2 ones

	// measure odometry 100 Hz - we update v, omega, bias
	// measure acceleration 500 Hz, input for the system <- that can be all updates
	// measure angular velocity 500 Hz, input for the system <- that can be all updates

	// it means 1133 updates per second, too much
	// we can use max 100
	// Kalman predicts to many variables, or too big matrices

	/* TODO main while loop should be moved HERE
	 *
	 */
	std::vector<int> liczby;
	liczby.push_back(16);
	liczby.push_back(20);
	liczby.push_back(-1);
	std::sort(liczby.begin(), liczby.end());
	std::priority_queue<int> pq;
}
