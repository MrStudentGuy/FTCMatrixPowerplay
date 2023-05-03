package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Autonomous.PositionCommand;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorHigh;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.TurretCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

    public static double maxVel = 500, maxAccel = 250, maxJerk = 0;
    TrajectorySequence autonomousTrajectory;

    @Override
    public void initialize() {
        elevator = new Elevator(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        slider = new Slider(this);
        drive = new Drive(hardwareMap, telemetry);
        guide = new Guide(hardwareMap, telemetry);
        endEffector = new EndEffector(hardwareMap, telemetry);

        drive.drive.setHardware(turret, elevator, slider, endEffector, guide);
//        timer = new ElapsedTime();



        Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
        drive.drive.setPoseEstimate(startPose);

        autonomousTrajectory = drive.drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> endEffector.closeGripper())
                .addTemporalMarker(0.25,()->elevator.extendTo(elevator.POSITIONS[elevator.MID_POLE]))
                .addTemporalMarker(0.4, ()->{turret.setDegree(145, maxVel, maxAccel, maxJerk);
                    endEffector.goAutoTop();
                    guide.outside();
                })
                .lineToLinearHeading(preloadDropPosition)
                .waitSeconds(0.05)
                .addTemporalMarker(()-> slider.setTargetSliderPosition(0.5))
                .waitSeconds(0.1)
//                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//                .waitSeconds(0.1)
                .addTemporalMarker(()->endEffector.goGripping())
                .addTemporalMarker(()-> guide.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {endEffector.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> guide.outside())
                .addTemporalMarker(()->slider.goInside())
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    guide.inside();
                    endEffector.closeGripper();})
                .addTemporalMarker( ()->{turret.setDegree(0, maxVel, maxAccel, maxJerk);})



                .addTemporalMarker(()->elevator.extendTo(elevator.AUTO_POSITION[4]))
                .lineToLinearHeading(pickPosition)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> endEffector.openGripper())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.goOutside())
                .waitSeconds(0.25)
                .addTemporalMarker(()-> endEffector.closeGripper())
                .waitSeconds(0.1)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .addTemporalMarker(()-> endEffector.goAutoTop())
                .addTemporalMarker(()-> guide.intermediate())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.goInside())
                .addTemporalMarker(()-> guide.outside())
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->turret.setDegree(144, maxVel, maxAccel, maxJerk))
                .lineToLinearHeading(farHighPosition,  SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> slider.goOutside())
//                .waitSeconds(0.3)
                .waitSeconds(0.6)
//                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//                .waitSeconds(0.1)
                .addTemporalMarker(()->endEffector.goGripping())
                .addTemporalMarker(()-> guide.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {endEffector.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> guide.outside())
                .addTemporalMarker(()->slider.goInside())
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    guide.inside();
                    endEffector.closeGripper();})
                .addTemporalMarker( ()->{turret.setDegree(0, maxVel, maxAccel, maxJerk);})




                .addTemporalMarker(()->elevator.extendTo(elevator.AUTO_POSITION[0]))
                .lineToLinearHeading(pickPosition)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> endEffector.openGripper())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.goOutside())
                .waitSeconds(0.25)
                .addTemporalMarker(()-> endEffector.closeGripper())
                .waitSeconds(0.1)
                .addTemporalMarker(()->elevator.extendTo(elevator.POSITIONS[elevator.HIGH_POLE]))
                .addTemporalMarker(()-> endEffector.goAutoTop())
                .addTemporalMarker(()-> guide.intermediate())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.goInside())
                .waitSeconds(0.2)
                .addTemporalMarker(()->turret.setDegree(-151, maxVel, maxAccel, maxJerk))
                .lineToLinearHeading(preloadDropPosition)
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()->guide.outside())
//                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> slider.goOutside())
//                .waitSeconds(0.3)
                .waitSeconds(0.6)
//                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//                .waitSeconds(0.1)
                .addTemporalMarker(()->endEffector.goGripping())
                .addTemporalMarker(()-> guide.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {endEffector.openGripper();})
                .waitSeconds(0.1)
                .addTemporalMarker(()-> guide.outside())
                .addTemporalMarker(()->slider.goInside())
                .waitSeconds(0.1)
                .addTemporalMarker( ()->{
                    guide.inside();
                    endEffector.closeGripper();})
                .addTemporalMarker( ()->{turret.setDegree(0, maxVel, maxAccel, maxJerk);})

//                .waitSeconds(10)


                .build();
    }


    @Override
    public void run() {
        super.run();
        if(timer == null){
            timer = new ElapsedTime();
            drive.drive.followTrajectorySequence(autonomousTrajectory);
        }
    }
}
