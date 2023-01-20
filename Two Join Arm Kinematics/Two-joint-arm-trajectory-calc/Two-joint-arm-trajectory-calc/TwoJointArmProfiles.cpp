#include "TwoJointArmProfiles.h"

TwoJointArmProfiles::TwoJointArmProfiles()
{
    hasProfiles_ = false;
}

void TwoJointArmProfiles::readProfiles()
{
    if (hasProfiles_)
    {
        return;
    }

    int k;
	for(int i = STOWED; i <= HIGH; ++i)
	{
        k = 0;
		for(int j = STOWED; j <= HIGH; ++j)
		{
			if(i == j)
			{
				continue;
			}

			pair<Positions, Positions> key{static_cast<Positions>(i), static_cast<Positions>(j)};
			map<double, pair<tuple<double, double, double>, tuple<double, double, double>>> profile;
            string fileName = to_string(i) + to_string(j) + ".csv";

            ifstream infile(fileName);

            string data;
            double time, omegaPos, omegaVel, omegaAcc, phiPos, phiVel, phiAcc;

            bool valid;
            std::size_t c1, c2, c3, c4, c5, c6;

            while (getline(infile, data))
            {
                valid = true;

                c1 = data.find(", ");
                if (c1 != string::npos)
                {
                    time = stod(data.substr(0, c1));

                    c2 = data.find(", ", c1 + 1);
                    if (c2 != string::npos)
                    {
                        omegaPos = stod(data.substr(c1 + 2, c2));

                        c3 = data.find(", ", c2 + 1);
                        if (c3 != string::npos)
                        {
                            omegaVel = stod(data.substr(c2 + 2, c3));

                            c4 = data.find(", ", c3 + 1);
                            if (c4 != string::npos)
                            {
                                omegaAcc = stod(data.substr(c3 + 2, c4));

                                c5 = data.find(", ", c4 + 1);
                                if (c5 != string::npos)
                                {
                                    phiPos = stod(data.substr(c4 + 2, c5));

                                    c6 = data.find(", ", c5 + 1);
                                    if (c6 != string::npos)
                                    {
                                        phiVel = stod(data.substr(c5 + 2, c6));
                                        phiAcc = stod(data.substr(c6 + 2));
                                    }
                                    else
                                    {
                                        valid = false;
                                    }
                                }
                                else
                                {
                                    valid = false;
                                }
                            }
                            else
                            {
                                valid = false;
                            }
                        }
                        else
                        {
                            valid = false;
                        }

                    }
                    else
                    {
                        valid = false;
                    }
                }
                else
                {
                    valid = false;
                }

                if (valid)
                {
                    tuple<double, double, double> omega{omegaPos, omegaVel, omegaAcc};
                    tuple<double, double, double> phi{phiPos, phiVel, phiAcc};
                    pair<tuple<double, double, double>, tuple<double, double, double>> fullAngPose{omega, phi};
                    pair<double, pair<tuple<double, double, double>, tuple<double, double, double>>> angPoint{time, fullAngPose};

                    profile.insert(angPoint);

                    ++k;
                }
            }
            
            pair<pair<Positions, Positions>, Profile> positionMapPoint{ key, profile };
            profiles_.insert(positionMapPoint);
            cout << "inserted " << k << " points" << endl;

            pair<Positions, Positions> testKey{ STOWED, HIGH };
            //cout << get<1>(profiles_.at(key).upper_bound(0.1)->second.second) << endl;
            //0.1001, -8.09238, 165.943, -2.04329, -1.03383, -20.4125, -10.328
            infile.close();
		}

	}

    hasProfiles_ = true;
}

std::pair<double, double> TwoJointArmProfiles::returnMaxTorque() {
    if (!hasProfiles_) {
        return {-1, -1};
    }

    std::pair<double, double> maxTorques = {-1, -1};

    for (pair<pair<Positions, Positions>, Profile> p : profiles_) { 
        std::pair<double, double> profileTorque = returnMaxTorqueOfProfile(p.second);
        maxTorques.first = max(profileTorque.first, maxTorques.first);
        maxTorques.second = max(profileTorque.second, maxTorques.second);
    }

}

/**
 * scan through all time stamps, find max torque
 * 
 * @param prof profile
*/
std::pair<double, double> TwoJointArmProfiles::returnMaxTorqueOfProfile(Profile prof) {
    std::pair<double, double> maxTorque = {0, 0};
    for(const auto& status : prof){
        auto value = status.second;
        auto xVals = value.first;
        auto yVals = value.second;
        std::pair<double, double> torqueAtTime = armKinematics::getTorque(
            std::get<0>(xVals),
            std::get<0>(yVals),
            std::get<2>(xVals),
            std::get<2>(yVals)
        );
        maxTorque.first = max(torqueAtTime.first, maxTorque.first);
        maxTorque.second = max(torqueAtTime.second, maxTorque.second);
    }
    return maxTorque;
}
