#pragma once
#include <tuple>
#include <map>
#include "constants.h"
#include <iostream>
#include <fstream>
#include <string>
#include "armKinematics.h"

//profile = time -> ((x, vx, ax), (y, vy, ay))
using Profile = map<double, pair<tuple<double, double, double>, tuple<double, double, double>>>;

class TwoJointArmProfiles
{
public:
	enum Positions
	{
		STOWED,
		GROUND_INTAKE,
		PLAYER_STATION,
		MID,
		HIGH
	};
	TwoJointArmProfiles();

	void readProfiles();
	std::pair<double, double> returnMaxTorque();
	std::pair<double, double> returnMaxTorqueOfProfile(map<double, pair<tuple<double, double, double>, tuple<double, double, double>>> profile);

	map<pair<Positions, Positions>, Profile> getProfiles();

private:
	map<pair<Positions, Positions>, Profile> profiles_; //Ok this is big but it makes the most sense at least to me

	bool hasProfiles_;
};

