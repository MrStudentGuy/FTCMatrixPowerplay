package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class ConfigurableAuto extends LinearOpMode {


    Pose2d startPose = new Pose2d(31.8, -63.3, Math.toRadians(0));
    final Pose2d dropPosition = new Pose2d(40, -12, Math.toRadians(0));
    final Pose2d pickingPosition1 = new Pose2d(45.01, -12, Math.toRadians(0));
    final Pose2d midDropPosition = new Pose2d(36, -10, Math.toRadians(0));

    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Robot robot = null;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);

        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        turret.reset();
        lift.reset();

        Servos.Slider.moveInside();
        Servos.Gripper.openGripper();
        Servos.Wrist.goGripping();
        Servos.AlignBar_2.goInside();
        Servos.SliderServo.setPosition(0);
        Robot.targetDegree = 0;
        robot.setPoseEstimate(startPose);


        TrajectorySequence startToMid = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.2)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.LOW_POLE],1))
                        .addTemporalMarker(()-> Servos.Slider.moveSlider(0.2))
                .lineToLinearHeading(midDropPosition)
                .UNSTABLE_addTemporalMarkerOffset(-1.3, ()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE],1))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->Robot.targetDegree = Auto2_0.preloadTurretPosition)
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> Servos.Wrist.setPosition(Auto2_0.preloadWristPosition))
//                .waitSeconds(0.01)
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Auto2_0.preloadAlignPosition))
                .addTemporalMarker(()-> robot.setTargetForSlider(Auto2_0.preloadSliderPosition))
                .waitSeconds(0.0001)
                .addTemporalMarker(()-> Servos.Gripper.openGripperFull())
                .addTemporalMarker(()-> Servos.AlignBar_2.goInside())
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setMaxPower(0.6))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                                .build();

        TrajectorySequence preloadToPick = robot.trajectorySequenceBuilder(startToMid.end())
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4],1))
                        .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.45))
                .waitSeconds(0.35)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                                .build();

        TrajectorySequence pickToDrop = robot.trajectorySequenceBuilder(pickingPosition1)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE],1))
                .waitSeconds(0.1)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.1)
                .addTemporalMarker(()->turret.setMaxPower(0.7))
                .addTemporalMarker(()->Robot.targetDegree = Auto2_0.highTurretPosition)
                .addTemporalMarker(()-> Servos.Wrist.setPosition(Auto2_0.highWristPosition))
                        .lineToLinearHeading(dropPosition)
                .waitSeconds(0.15)
                .addTemporalMarker(()-> Servos.AlignBar_2.setPosition(Auto2_0.highAlignPosition))
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 2.8)
                .addTemporalMarker(()->robot.setTargetForSlider(Auto2_0.highSliderPosition))
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.15)
                .addTemporalMarker(()-> Servos.Gripper.openGripperFull())
                .addTemporalMarker(()-> Servos.AlignBar_2.goInside())
                .addTemporalMarker(()->Robot.sliderMaxAcceleration = 100)
                .addTemporalMarker(()->robot.setTargetForSlider(0))
                .waitSeconds(0.2)
                .addTemporalMarker(()->turret.setMaxPower(0.9))
                .addTemporalMarker(()->Robot.targetDegree = 0)
                                .build();

        TrajectorySequence dropToPick3 = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3],1))
                        .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.45))
                .waitSeconds(0.3)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                                .build();


        TrajectorySequence dropToPick2 = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.45))
                .waitSeconds(0.3)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();


        TrajectorySequence dropToPick1 = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.45))
                .waitSeconds(0.3)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();


        TrajectorySequence dropToPick0 = robot.trajectorySequenceBuilder(dropPosition)
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[0],1))
                .lineToLinearHeading(pickingPosition1)
                .addTemporalMarker(()->robot.setTargetForSlider(0.45))
                .waitSeconds(0.3)
                .addTemporalMarker(()-> robot.setTargetForSlider(0.78))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .build();

        waitForStart();

        robot.followTrajectorySequence(startToMid);
        robot.followTrajectorySequence(preloadToPick);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick3);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick2);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick1);
        robot.followTrajectorySequence(pickToDrop);
        robot.followTrajectorySequence(dropToPick0);
        robot.followTrajectorySequence(pickToDrop);
        double turretAngle = turret.getDegree();
        double height = lift.getPosition()[0];
        boolean prevStart = false;

        double alignBarPos = Servos.AlignBar_2.getPosition();
        double wristPos = Servos.Wrist.getPosition();
        double x = Servos.Slider.getPosition();

        while(opModeIsActive()){
            robot.update();

            if(gamepad1.a){
                alignBarPos += 0.01;
            }
            else if(gamepad1.b){
                alignBarPos -= 0.01;
            }

            if(gamepad1.x){
                wristPos += 0.01;
            }
            else if(gamepad1.y){
                wristPos -= 0.01;
            }

            if(gamepad1.dpad_up){
                height += 5;
            }
            else if(gamepad1.dpad_down){
                height -= 5;
            }

            if(gamepad1.dpad_right){
                turretAngle -= 0.5;
            }
            else if(gamepad1.dpad_left){
                turretAngle += 0.5;
            }

            if(gamepad1.right_bumper){
                x += 0.01;
            }
            else if(gamepad1.left_bumper){
                x -= 0.01;
            }

            if(gamepad1.start && prevStart == false){
                Servos.Gripper.toggle();
            }
            prevStart = gamepad1.start;


            if(gamepad1.right_stick_button){
                robot.followTrajectorySequence(pickToDrop);
            }

            if(gamepad1.left_stick_button){
                robot.followTrajectorySequence(dropToPick2);
            }

            robot.setTargetForSlider(x);
            Servos.Wrist.setPosition(wristPos);
            Servos.AlignBar_2.setPosition(alignBarPos);
            Robot.targetDegree = turretAngle;
            lift.extendTo((int)height, 1);

            telemetry.addData("Wrist: ", wristPos);
            telemetry.addData("Align: ", alignBarPos);
            telemetry.addData("Height: ", height);
            telemetry.addData("Angle: ", turretAngle);
            telemetry.addData("Currents Lift: ", lift.getCurrent()[0] + ", " + lift.getCurrent()[1]);
            telemetry.addData("X: ", x);



        }
    }
}
