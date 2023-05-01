package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Autonomous.PositionCommand;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorHigh;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.subsystems.Guide;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Autonomous_Left extends CommandOpMode {

    final Pose2d preloadDropPosition = new Pose2d(-40.5, -12, Math.toRadians(180));
    final Pose2d pickPosition = new Pose2d(-46.2, -12, Math.toRadians(180));

    final Pose2d farHighPosition = new Pose2d(-16.5, -12, Math.toRadians(180));


    Elevator elevator;
    Turret turret;
    Slider slider;
    Drive drive;
    Guide guide;
    EndEffector endEffector;

    ElapsedTime timer;


    @Override
    public void initialize() {
        elevator = new Elevator(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        slider = new Slider(this);
        drive = new Drive(hardwareMap, telemetry);
        guide = new Guide(hardwareMap, telemetry);
        endEffector = new EndEffector(hardwareMap, telemetry);
        timer = new ElapsedTime();

        Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
        drive.drive.setPoseEstimate(startPose);

        TrajectorySequence goToCenter =drive.drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(preloadDropPosition)
                                .build();

        schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drive, goToCenter),
                        new ElevatorHigh(elevator),
                        new TurretCommand(turret, elevator, 180)
                )

        );
    }


    @Override
    public void run() {
        super.run();
    }
}
