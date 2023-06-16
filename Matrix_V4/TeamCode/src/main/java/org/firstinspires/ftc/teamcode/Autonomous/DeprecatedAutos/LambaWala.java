//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import org.firstinspires.ftc.teamcode.Subsystems.Servos;
//import org.firstinspires.ftc.teamcode.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Autonomous
//public class LambaWala extends LinearOpMode {
//    boolean lastUp, lastDown, lastA, lastB, lastRB, lastLB;
//    ElapsedTime timer = null;
//    double sliderPos = 0;
//    double wristPos = 0;
//    double alignBarPos = 0;
//
//    final Pose2d coneDropPosition = new Pose2d(-42, -12, Math.toRadians(180));
//    final Pose2d pickingPosition = new Pose2d(-46, -12, Math.toRadians(180));
//
//    Lift lift = null;
//    Servos servos = null;
//    Turret turret = null;
//    Robot robot = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//timer = new ElapsedTime();
//        lift = new Lift(hardwareMap, telemetry);
//        servos = new Servos(hardwareMap, telemetry);
//        turret = new Turret(hardwareMap, "turret", telemetry);
//
//        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//
//        turret.reset();
//        lift.reset();
//
//
//        alignBarPos = Servos.AlignBar.getPosition();
//        sliderPos = Servos.Slider.getPosition();
//        wristPos = Servos.Wrist.getPosition();
//        Servos.Slider.moveInside();
//        Servos.Gripper.openGripper();
//        Servos.Wrist.goGripping();
//        Servos.AlignBar.inside();
//        turret.setTargetDegree(0);
//
//        final Pose2d preloadDropPosition = new Pose2d(-40.5, -12, Math.toRadians(180));
//
//        Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
//        robot.setPoseEstimate(startPose);
//
//
//
//        TrajectorySequence autonomousTrajectory = robot.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
//                .addTemporalMarker(0.25,()->lift.extendTo(lift.POSITIONS[lift.MID_POLE], 1))
//                .addTemporalMarker(0.4, ()->{turret.setTargetDegree(145);
//                    Servos.Wrist.goAutoTop();
//                    Servos.AlignBar.outside();
//                })
//                .lineToLinearHeading(preloadDropPosition)
//                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
////                .waitSeconds(0.05)
////                .UNSTABLE_addTemporalMarkerOffset(-0.001,()-> Servos.Slider.moveSlider(0.5))
//                .waitSeconds(0.2)
////                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.25))
//                .waitSeconds(0.03)
//                .addTemporalMarker(()->Servos.Wrist.goGripping())
//                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
//                .waitSeconds(0.1)
//                .addTemporalMarker( ()-> {
//                    Servos.Gripper.openGripper();})
//                .addTemporalMarker(()-> Servos.AlignBar.outside())
//                .addTemporalMarker(()->Servos.Slider.moveInside())
//                .waitSeconds(0.1)
//                .addTemporalMarker( ()->{
//                    Servos.AlignBar.inside();
//                    Servos.Gripper.closeGripper();})
//                .waitSeconds(0.1)
//                .addTemporalMarker( ()->{turret.setTargetDegree(0);})
//                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//                .lineToLinearHeading(pickingPosition)
//                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
//
//                .build();
//        TrajectorySequence drop = robot.trajectorySequenceBuilder(autonomousTrajectory.end())
//                        .lineToLinearHeading(coneDropPosition)
//                                .build();
//
//        TrajectorySequence pick = robot.trajectorySequenceBuilder(drop.end())
//                        .lineToLinearHeading(pickingPosition)
//                                .build();
//
//
//
//
//        waitForStart();
//        timer.reset();
////        Servos.Gripper.closeGripper();
////        delay(1000);
////        lift.extendTo(lift.POSITIONS[lift.MID_POLE], 1);
////        delay(1000);
//        robot.followTrajectorySequence(autonomousTrajectory);
//        timer.reset();
//
//        for(int i=4;i>=0;i--) {
//            delay(500);
//            Servos.Gripper.openGripper();
//            delay(200);
//            Servos.Slider.moveOutside();
//            delay(600);
//            Servos.Gripper.closeGripper();
//            delay(500);
//            lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1);
//            delay(200);
//            Servos.Wrist.setPosition(0.3279);
//            Servos.Slider.moveInside();
//            delay(600);
//            turret.setTargetDegree(-153);
//            robot.followTrajectorySequence(drop);
//            Servos.AlignBar.moveTo(0.206);
//            delay(1500);
//            Servos.Slider.moveSlider(0.28);
//            delay(1000);
//            Servos.Wrist.goGripping();
//            delay(500);
//            Servos.Slider.moveInside();
//            delay(100);
//            Servos.Gripper.openGripper();
//            delay(50);
//            Servos.Slider.moveInside();
//            delay(100);
//            Servos.AlignBar.inside();
//            Servos.Gripper.closeGripper();
//            delay(750);
//            turret.setTargetDegree(0);
//            robot.followTrajectorySequence(pick);
//            delay(900);
//            if(i>0)
//            lift.extendTo(lift.AUTO_POSITION[i-1], 1);
//            delay(1000);
//
//        }
//        delay(10000);
//
//        alignBarPos = Servos.AlignBar.getPosition();
//        sliderPos = Servos.Slider.getPosition();
//        wristPos = Servos.Wrist.getPosition();
//
//
//
//
//
//
//        while(opModeIsActive()){
//            if(gamepad1.dpad_up && !lastUp){
//                wristPos+=0.009;
//            }
//            else if(gamepad1.dpad_down && !lastDown){
//                wristPos-=0.009;
//            }
//
//            if(gamepad1.a && !lastA){
//                alignBarPos+=0.009;
//            }
//            else if(gamepad1.y && !lastB){
//                alignBarPos -= 0.009;
//            }
//
//            if(gamepad1.right_bumper && !lastRB){
//                sliderPos+=0.009;
//            }
//            else if(gamepad1.left_bumper && !lastLB){
//                sliderPos-=0.009;
//            }
//
//            lastRB = gamepad1.right_bumper;
//            lastUp = gamepad1.dpad_up;
//            lastLB = gamepad1.left_bumper;
//            lastB = gamepad1.y;
//            lastA = gamepad1.a;
//            lastDown = gamepad1.dpad_down;
//
//            if(gamepad1.start){
//                Servos.Wrist.goGripping();
//                delay(1000);
//                Servos.Gripper.openGripper();
//            }
//            telemetry.addData("Turret: ", turret.getDegree());
//            telemetry.addData("Align Bar: ", alignBarPos);
//            telemetry.addData("Wrist: ", wristPos);
//            telemetry.addData("Slider: ", sliderPos);
//            telemetry.update();
//            Servos.AlignBar.moveTo(alignBarPos);
//            Servos.Wrist.setPosition(wristPos);
//            Servos.Slider.moveSlider(sliderPos);
//        }
//    }
//    void delay(double ms){
//        while(timer.milliseconds() < ms){
//            robot.update();
//        }
//        timer.reset();
//    }
//}
