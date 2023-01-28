#include "AutoPaths.h"

AutoPaths::AutoPaths(SwerveDrive *swerveDrive) : swerveDrive_(swerveDrive)
{
    pointNum_ = 0;
    actionNum_ = 0;
    dumbTimerStarted_ = false;
    actionsSet_ = false;
    pathSet_ = false;
    pathGenerated_ = false;
    mirrored_ = false;
}

void AutoPaths::setPath(Path path)
{
    path_ = path;
    pointNum_ = 0;
    swervePoints_.clear();
    nextPointReady_ = false;
    dumbTimerStarted_ = false;
    failsafeStarted_ = false;

    switch (path_)
    {
    case BIG_BOY:
    {
        swervePoints_.push_back(SwervePose(0, 1, 0, 0));
        swervePoints_.push_back(SwervePose(1, 1, 0, 0.1));
        swervePoints_.push_back(SwervePose(1, 0, 90, 1));
        swervePoints_.push_back(SwervePose(0, 1, 0, 10));
        swervePoints_.push_back(SwervePose(0, 0, -90, 1));

        break;
    }
    case PRELOADED_CONE:
    {
        swervePoints_.push_back(SwervePose(0, 0, 0, 0));
        break;
    }
    case PRELOADED_CUBE:
    {
        swervePoints_.push_back(SwervePose(0, 0, 0, 0));
        break;
    }
    case FIRST_CONE:
    {
        double deltaX = 0.4064;
        if (mirrored_)
        {
            deltaX *= -1;
        }

        swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));
        swervePoints_.push_back(SwervePose(0, 0, 0, 0));
        break;
    }
    case FIRST_CUBE:
    {
        double deltaX = 0.4064;
        if (mirrored_)
        {
            deltaX *= -1;
        }

        swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));

        double returnX = 0.555752;
        if (mirrored_)
        {
            returnX *= -1;
        }
        swervePoints_.push_back(SwervePose(0, returnX, 0, 0));
        break;
    }
    case SECOND_CONE:
    {
        // TODO curves woo
        double deltaX = 0.4064;
        if (mirrored_)
        {
            deltaX *= -1;
        }

        swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));
        double deltaX2 = 1.2192;
        double yaw = 90;
        if (mirrored_)
        {
            deltaX2 *= -1;
            yaw *= -1;
        }
        
        swervePoints_.push_back(SwervePose(deltaX+deltaX2, 5.2451, yaw, 0.8));
        swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0.8));
        swervePoints_.push_back(SwervePose(0, 0, 0, 0));

        break;
    }
    case SECOND_CUBE:
    {
        // TODO curves woo
        double deltaX = 0.4064;
        if (mirrored_)
        {
            deltaX *= -1;
        }

        swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));
        double deltaX2 = 1.2192;
        double yaw = 90;
        if (mirrored_)
        {
            deltaX2 *= -1;
            yaw *= -1;
        }
        
        swervePoints_.push_back(SwervePose(deltaX+deltaX2, 5.2451, yaw, 0.8));
        swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0.8));

        double returnX = 0.555752;
        if (mirrored_)
        {
            returnX *= -1;
        }
        swervePoints_.push_back(SwervePose(0, returnX, 0, 0));

        break;
    }
    case AUTO_DOCK:
    {
        double deltaX = 2.21869;
        if (mirrored_)
        {
            deltaX *= -1;
        }

        swervePoints_.push_back(SwervePose(deltaX, 1.033526, 0, 0));
        break;
    }
    case NOTHING:
    {
        break;
    }
    case DRIVE_BACK_DUMB:
    {
        break;
    }
    }

    pathSet_ = true;
    pathGenerated_ = false;
}

AutoPaths::Path AutoPaths::getPath()
{
    return path_;
}

void AutoPaths::setActions(Path a1, Path a2, Path a3)
{
    actions_.clear();
    actions_.push_back(a1);
    actions_.push_back(a2);
    actions_.push_back(a3);

    actionNum_ = 0;
    pointNum_ = 0;
    swervePoints_.clear();
    nextPointReady_ = false;
    dumbTimerStarted_ = false;
    failsafeStarted_ = false;

    actionsSet_ = true;
    pathGenerated_ = false;
}

void AutoPaths::startTimer()
{
    startTime_ = timer_.GetFPGATimestamp().value();
}

void AutoPaths::setActionsSet(bool actionsSet)
{
    actionsSet_ = actionsSet;
}

void AutoPaths::setPathSet(bool pathSet)
{
    pathSet_ = pathSet;
}

void AutoPaths::periodic(double yaw, SwerveDrive *swerveDrive)
{
    if (!actionsSet_)
    {
        return;
    }

    if (!pathSet_)
    {
        if (actionNum_ > 2)
        {
            swerveDrive->drive(0, 0, 0);
            return;
        }

        setPath(actions_[actionNum_]);
    }

    // frc::SmartDashboard::PutBoolean("actions set", actionsSet_);
    // frc::SmartDashboard::PutBoolean("path set", pathSet_);
    // frc::SmartDashboard::PutNumber("action num", actionNum_);
    // frc::SmartDashboard::PutNumber("point num", pointNum_);

    double time = timer_.GetFPGATimestamp().value() - startTime_;
    // frc::SmartDashboard::PutNumber("time", time);

    bool pathOver = false;
    bool pointOver = false;
    if (path_ != DRIVE_BACK_DUMB && path_ != NOTHING)
    {
        SwervePose *pose = nullptr;
        for (size_t i = pointNum_; i < swervePoints_.size(); ++i)
        {
            // pose = swervePoints_[i].getPose(time, pointOver);
            if (!pathGenerated_)
            {
                SwervePose currPose(swerveDrive_->getX(), swerveDrive_->getY(), -swerveDrive_->getYaw(), 0);

                currPath_.reset();
                currPath_.addPoint(currPose);
                currPath_.addPoint(swervePoints_[i]);

                currPath_.generateTrajectory(false);

                pathGenerated_ = true;
            }

            pose = currPath_.getPose(time, pointOver);

            if (!pointOver)
            {
                break;
            }
            else
            {
                if (nextPointReady_ && i == swervePoints_.size() - 1)
                {
                    // frc::SmartDashboard::PutBoolean("f", true);
                    pathSet_ = false;
                    nextPointReady_ = false;
                    ++actionNum_;
                    startTimer();
                    return;
                }
                else if (i == swervePoints_.size() - 1)
                {
                    pathOver = true;
                }
                else if (nextPointReady_ && i != swervePoints_.size() - 1)
                {
                    nextPointReady_ = false;
                    ++pointNum_;
                    startTimer();
                    // frc::SmartDashboard::PutNumber("what", 1);
                    pathGenerated_ = false;
                    time = timer_.GetFPGATimestamp().value() - startTime_;
                }
                else
                {
                    break;
                }
            }
        }

        if (pose != nullptr)
        {
            swerveDrive->drivePose(yaw, *pose);
            delete pose;
        }
    }
    else
    {
        if (!dumbTimerStarted_)
        {
            timer_.Stop();
            timer_.Reset();
            timer_.Start();
            dumbTimerStarted_ = true;
        }
    }

    switch (path_)
    {
    case BIG_BOY:
    {
        if (pathOver)
        {
            // Do a thing
        }

        if (pointOver)
        {
            switch (pointNum_)
            {
            case 0:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 2)
                {
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeStarted_ = false;
                    nextPointReady_ = true;
                }
                break;
            }
            case 1:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 2)
                {
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeStarted_ = false;
                    nextPointReady_ = true;
                }
                break;
            }
            case 2:
            {
                nextPointReady_ = true;
                break;
            }
            case 3:
            {
                nextPointReady_ = true;
                break;
            }
            case 4:
            {
                nextPointReady_ = true;
                break;
            }
            }
        }

        break;
    }
    case PRELOADED_CONE:
    {
        // TODO place cone
        pointOver = true;
        nextPointReady_ = true;
        break;
    }
    case PRELOADED_CUBE:
    {
        // TODO place cones
        pointOver = true;
        nextPointReady_ = true;
        break;
    }
    case FIRST_CONE:
    {
        if (pointOver && pointNum_ == 0)
        {
            nextPointReady_ = true;
        }

        // TODO place cone
        // if done placing
        if (/*done placing &&*/ pointOver && pointNum_ == 1)
        {
            nextPointReady_ = true;
        }

        break;
    }
    case FIRST_CUBE:
    {
        if (pointOver && pointNum_ == 0)
        {
            nextPointReady_ = true;
        }

        // TODO place cone
        // if done placing
        if (/*done placing &&*/ pointOver && pointNum_ == 1)
        {
            nextPointReady_ = true;
        }

        break;
    }
    case SECOND_CONE:
    {
        // TODO see if curves are the same
        pointOver = true;
        nextPointReady_ = true;
        break;
    }
    case SECOND_CUBE:
    {
        //TODO see if curves are the same
        pointOver = true;
        nextPointReady_ = true;
        break;
    }
    case AUTO_DOCK:
    {
        if (pointOver)
        {
            // TODO auto dock
        }

        break;
    }
    case NOTHING:
    {
        swerveDrive_->drive(0, 0, 0);
        break;
    }
    case DRIVE_BACK_DUMB:
    {
        if (!failsafeStarted_)
        {
            failsafeStarted_ = true;
            failsafeTimer_.Stop();
            failsafeTimer_.Reset();
            failsafeTimer_.Start();
        }

        if (failsafeTimer_.Get().value() < 2)
        {
            swerveDrive_->drive(0, 0.2, 0);
        }
        else
        {
            swerveDrive_->drive(0, 0, 0);
        }

        break;
    }
    }
}

double AutoPaths::initYaw()
{
    // switch (path_)
    // {
    // case BIG_BOY:
    // {
    //     return 0;
    //     break;
    // }
    // }

    return 0;
}

int AutoPaths::pointNum()
{
    return pointNum_;
}

void AutoPaths::setMirrored(bool mirrored)
{
    mirrored_ = mirrored;
}