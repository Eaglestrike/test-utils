#include "TwoJointArmProfiles.h"

TwoJointArmProfiles::TwoJointArmProfiles()
{
    hasProfiles_ = false;
}

void TwoJointArmProfiles::readProfiles()
{
	for(int i = STOWED; i < HIGH; ++i)
	{
		for(int j = STOWED; i < HIGH; ++i)
		{
			if(i == j)
			{
				break;
			}

			pair<Positions, Positions> key{static_cast<Positions>(i), static_cast<Positions>(j)};
			map<double, pair<tuple<double, double, double>, tuple<double, double, double>>> profile;
            string fileName = to_string(i) + to_string(j) + ".csv";

            if (hasProfiles_)
            {
                return;
            }
            ifstream infile(fileName);

            string data;
            double distance, angle, velocity, partDer;

            bool valid;
            std::size_t c1, c2, c3;

            while (getline(infile, data))
            {
                valid = true;

                c1 = data.find(", ");
                if (c1 != string::npos)
                {
                    distance = stod(data.substr(0, c1));

                    c2 = data.find(", ", c1 + 1);
                    if (c2 != string::npos)
                    {
                        angle = stod(data.substr(c1 + 2, c2));

                        c3 = data.find(", ", c2 + 1);
                        if (c3 != string::npos)
                        {
                            velocity = stod(data.substr(c2 + 2, c3));
                            partDer = stod(data.substr(c3 + 2));
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
                    tuple<double, double, double> shotData(angle, velocity, partDer);
                    pair<double, tuple<double, double, double>> distancePoint(distance, shotData);

                    profile.insert(distancePoint);
                }
            }

            //hasMap_ = true;
            //frc::SmartDashboard::PutNumber("Map Points", mapPoints_);
            infile.close();

		}
	}
}