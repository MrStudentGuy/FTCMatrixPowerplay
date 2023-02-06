package org.firstinspires.ftc.teamcode.drive.Localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public interface Localizer {
    void periodic();

    Pose2d getPose();

    void setPose(Pose2d pose);
}
