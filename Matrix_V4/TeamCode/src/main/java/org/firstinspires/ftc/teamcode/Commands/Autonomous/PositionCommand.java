package org.firstinspires.ftc.teamcode.Commands.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class PositionCommand extends CommandBase {


    Drive drive;
    TrajectorySequence seq;
    public PositionCommand(Drive drive, TrajectorySequence sequence){
        this.drive = drive;
        addRequirements(drive);
        this.seq = sequence;
    }

    @Override
    public void initialize() {
        super.initialize();
        drive.drive.followTrajectorySequenceAsync(seq);
    }

    @Override
    public boolean isFinished() {
        return !drive.drive.isBusy();
    }
}
