package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
public class RedLeft extends LinearOpMode {
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    AnalogInput sensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        sensor = hardwareMap.get(AnalogInput.class, "ultrasound1");
        lift.reset();
        turret.reset();
        setInitialPositions();


        double V = sensor.getVoltage();
        double d = V * 520.00/3.3;

        telemetry.addData("d: ", d);
        double x = -72.00 + d;
        double inch = x/2.54;
        telemetry.addData("x: ", inch);
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence startToDropPreload =drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setDegree(-90))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
//                .addTemporalMarker(()->)

                                .build();

        TrajectorySequence pickCone5 = drive.trajectorySequenceBuilder(startToDropPreload.end())
                .lineToConstantHeading(new Vector2d(-36, -12))
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
                        .lineToConstantHeading(new Vector2d(-57, -12))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                                .build();


        TrajectorySequence dropCone5 = drive.trajectorySequenceBuilder(pickCone5.end())
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                        .build();

        TrajectorySequence pickCone4 = drive.trajectorySequenceBuilder(dropCone5.end())
//                .lineToConstantHeading(new Vector2d(-36, -12))
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))
                .lineToConstantHeading(new Vector2d(-57, -12))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence dropCone4 = drive.trajectorySequenceBuilder(pickCone4.end())
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone3 = drive.trajectorySequenceBuilder(dropCone4.end())
//                .lineToConstantHeading(new Vector2d(-36, -12))
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))
                .lineToConstantHeading(new Vector2d(-57, -12))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence dropCone3 = drive.trajectorySequenceBuilder(pickCone3.end())
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone2 = drive.trajectorySequenceBuilder(dropCone3.end())
//                .lineToConstantHeading(new Vector2d(-36, -12))
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))
                .lineToConstantHeading(new Vector2d(-57, -12))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence dropCone2 = drive.trajectorySequenceBuilder(pickCone2.end())
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .waitSeconds(0.05)
                .build();

        TrajectorySequence pickCone1 = drive.trajectorySequenceBuilder(dropCone2.end())
//                .lineToConstantHeading(new Vector2d(-36, -12))
                .addTemporalMarker(()->turret.setDegree(90))
                .addTemporalMarker(()->Servos.Slider.moveHalfway())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))
                .lineToConstantHeading(new Vector2d(-57, -12))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .build();

        TrajectorySequence dropCone1 = drive.trajectorySequenceBuilder(pickCone1.end())
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.4)
                .addTemporalMarker(()->turret.setDegree(0))
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .lineToConstantHeading(new Vector2d(-24, -12.31))
//                .waitSeconds(0.01)
//                .lineToConstantHeading(new Vector2d(-36, 0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .build();



        waitForStart();
        lift.reset();
        drive.followTrajectorySequence(startToDropPreload);
        drive.followTrajectorySequence(pickCone5);
        drive.followTrajectorySequence(dropCone5);
        drive.followTrajectorySequence(pickCone4);
        drive.followTrajectorySequence(dropCone4);
        drive.followTrajectorySequence(pickCone3);
        drive.followTrajectorySequence(dropCone3);
        drive.followTrajectorySequence(pickCone2);
        drive.followTrajectorySequence(dropCone2);
        drive.followTrajectorySequence(pickCone1);
        drive.followTrajectorySequence(dropCone1);

        while(opModeIsActive()){

        }
    }

    private void setInitialPositions(){
        lift.extendTo(0, 1);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        turret.setDegree(0);
    }
}
