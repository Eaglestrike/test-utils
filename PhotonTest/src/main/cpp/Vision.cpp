#include "Vision.h"

photonlib::PhotonPipelineResult Vision::latestResult() {
    return camera.GetLatestResult();
}

double Vision::getLatency(){
    return latestResult().GetLatency().value();
}

frc::Transform3d Vision::getPose() {
    return latestResult().GetBestTarget().GetBestCameraToTarget();
}

