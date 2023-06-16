//package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Subsystems.Lift;
//import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
//import org.firstinspires.ftc.teamcode.Subsystems.Servos;
//
//@Autonomous
//public class testAutoIntake extends LinearOpMode {
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Sensors sensors = new Sensors(hardwareMap, telemetry);
//        Servos servos = new Servos(hardwareMap, telemetry);
//        Lift lift = new Lift(hardwareMap, telemetry);
//        ElapsedTime timer0 = new ElapsedTime();
//        ElapsedTime timer1 = new ElapsedTime();
//
//        boolean flag0 = false, flag1 = false;
//
//        Servos.Wrist.goGripping();
//        Servos.Gripper.openGripper();
//        lift.extendToGrippingPosition();
//        waitForStart();
//
//        while(opModeIsActive()){
//            if(Sensors.GripperSensor.checkRed() && Sensors.GripperSensor.getDistanceMM() < 22){
//                Servos.Gripper.closeGripper();
//                telemetry.addLine("RED CONE HAS BEEN DETECTED IN INTAKE");
//                if(!flag0){
//                    timer0.reset();
//                    flag0 = true;
//                }
//                if(timer0.milliseconds() > 1000){
//                    lift.extendTo(0, 1);
//                    if(timer0.milliseconds() > 1500){
//                        Servos.Wrist.goInit();
//                    }
//                }
//            }
//
//            if(gamepad1.right_bumper){
//                Servos.Gripper.openGripper();
//                Servos.Wrist.goGripping();
//                lift.extendToGrippingPosition();
//                flag0 = false;
//            }
//            Sensors.GripperSensor.printRGBDistance();
//            telemetry.update();
//        }
//    }
//}
