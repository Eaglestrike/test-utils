#pragma once
#include "photonlib/PhotonCamera.h"

class Vision{
    public:
        photonlib::PhotonCamera camera{"gloworm"}; //name can be found on website
        photonlib::PhotonPipelineResult latestResult();
        frc::Transform3d getPose();
        double getLatency();

};