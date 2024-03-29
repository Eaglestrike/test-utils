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
    for (int i = STOWED; i <= INTAKE; ++i)
    {
        k = 0;
        for (int j = STOWED; j <= INTAKE; ++j)
        {
            if (i == j)
            {
                continue;
            }

            pair<Positions, Positions> key{ static_cast<Positions>(i), static_cast<Positions>(j) };
            map<double, pair<tuple<double, double, double>, tuple<double, double, double>>> profile;
            string fileName = to_string(i) + to_string(j) + ".csv";

            ifstream infile(fileName);

            string data;
            double time, thetaPos, thetaVel, thetaAcc, phiPos, phiVel, phiAcc;

            bool valid;
            size_t c1, c2, c3, c4, c5, c6;

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
                        thetaPos = stod(data.substr(c1 + 2, c2));

                        c3 = data.find(", ", c2 + 1);
                        if (c3 != string::npos)
                        {
                            thetaVel = stod(data.substr(c2 + 2, c3));

                            c4 = data.find(", ", c3 + 1);
                            if (c4 != string::npos)
                            {
                                thetaAcc = stod(data.substr(c3 + 2, c4));

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
                    tuple<double, double, double> theta{ thetaPos, thetaVel, thetaAcc };
                    tuple<double, double, double> phi{ phiPos, phiVel, phiAcc };
                    pair<tuple<double, double, double>, tuple<double, double, double>> fullAngPose{ theta, phi };
                    pair<double, pair<tuple<double, double, double>, tuple<double, double, double>>> angPoint{ time, fullAngPose };

                    profile.insert(angPoint);

                    ++k;
                }
            }

            pair<pair<Positions, Positions>, map<double, pair<tuple<double, double, double>, tuple<double, double, double>>>> positionMapPoint{ key, profile };
            profiles_.insert(positionMapPoint);
            //cout << "inserted " << k << " points" << endl;

            //pair<Positions, Positions> testKey{ STOWED, HIGH };
            //cout << get<1>(profiles_.at(key).upper_bound(0.1)->second.second) << endl;
            //0.1001, -8.09238, 165.943, -2.04329, -1.03383, -20.4125, -10.328
            infile.close();
        }

    }

    hasProfiles_ = true;
}

tuple<double, double, double> TwoJointArmProfiles::getThetaProfile(pair<Positions, Positions> key, double time)
{
    auto profile = profiles_.at(key).upper_bound(time);
    if(profile == profiles_.at(key).end())
    {
        --profile;
        return tuple<double, double, double>{get<0>(profile->second.first), 0, 0};
    }

    --profile;
   return profile->second.first;
}

tuple<double, double, double> TwoJointArmProfiles::getPhiProfile(pair<Positions, Positions> key, double time)
{
    auto profile = profiles_.at(key).upper_bound(time);
    if (profile == profiles_.at(key).end())
    {
        --profile;
        return tuple<double, double, double>{get<0>(profile->second.second), 0, 0};
    }

    --profile;
    return profile->second.second;
    /*tuple<double, double, double> profile = profiles_.at(key).upper_bound(time)->second.second;

    if (profiles_.at(key).upper_bound(time) == profiles_.at(key).end())
    {
        return tuple<double, double, double>{get<0>(profile), 0, 0};
    }
    else
    {
        return profile;
    }*/
}