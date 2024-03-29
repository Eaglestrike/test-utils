#pragma once

#include <frc/Timer.h>

#include "SwerveDrive.h"
#include "SwervePath.h"
#include "Constants.h"
#include <vector>

class AutoPaths
{
    public:
        enum Path
        {
            BIG_BOY,
            PRELOADED_CONE,
            PRELOADED_CUBE,
            FIRST_CONE,
            FIRST_CUBE,
            SECOND_CONE,
            SECOND_CUBE,
            AUTO_DOCK,
            NOTHING,
            DRIVE_BACK_DUMB
        };
        AutoPaths(SwerveDrive* swerveDrive);
        void setActions(Path a1, Path a2, Path a3);
        vector<Path> getActions();

        void startTimer();
        void setActionsSet(bool actionsSet);
        void setPathSet(bool pathSet);

        void periodic(double yaw, SwerveDrive* swerveDrive);
        double initYaw();

        int pointNum();

        void setMirrored(bool mirrored);
    private:
        vector<Path> actions_;
        Path path_;
        SwerveDrive* swerveDrive_;

        frc::Timer timer_;
        frc::Timer failsafeTimer_;
        double startTime_;
        bool nextPointReady_, failsafeStarted_, dumbTimerStarted_, pathSet_, pathGenerated_, actionsSet_, mirrored_;

        //vector<SwervePath> swervePaths_;
        int actionNum_;
        vector<SwervePose> swervePoints_;
        SwervePath currPath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};
        int pointNum_;

        void setPath(Path path);
        Path getPath();
};