package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class    Servos {
    static Servo GripperServo;

    static Servo AlignServo2;
    static Servo WristServo;
    public static Servo SliderServo;
    static Servo AlignServo;

    public static double gripperClosePosition = 0.4;
    public static double gripperOpenPosition = 0.68;

    public static double gripperInitPosition = 0.7;

    public static double TopPosition = 0.27;
    public static double InitPosition = 0.05;
    public static double GrippingPosition = 0.01;
    public static double TopAutoPosition = 0.4;




    public Servos(HardwareMap hardwareMap, Telemetry telemetry){
        GripperServo = hardwareMap.get(Servo.class, "Gripper");
        WristServo = hardwareMap.get(Servo.class, "Wrist");
        SliderServo = hardwareMap.get(Servo.class, "Slider");
        SliderServo.setDirection(Servo.Direction.REVERSE);
        AlignServo = hardwareMap.get(Servo.class, "Align"); //auto stop pole shaking, goBilda servo
        AlignServo2 = hardwareMap.get(Servo.class, "Align2");
        WristServo.setDirection(Servo.Direction.REVERSE);
    }

    public static class Gripper {
        public static String gripperState = "OPEN";

        private static final double gripperBeaconPosition = 0.4;

        public static void openGripper() {
            gripperState = "OPEN";
            GripperServo.setPosition(gripperOpenPosition);
        }

        public static void update(){
            if(gripperState == "OPEN"){
                GripperServo.setPosition(0.71);
            }
            else if(gripperState == "CLOSED"){
                GripperServo.setPosition(gripperClosePosition);
            }
        }
        public static void openGripperFull(){
            gripperState = "OPEN";
            GripperServo.setPosition(0.71);
        }

        public static void closeGripper() {
            gripperState = "CLOSED";
            GripperServo.setPosition(gripperClosePosition);
        }

        public static void gripBeacon(){
            gripperState = "OPEN";
            GripperServo.setPosition(gripperBeaconPosition);
        }

        public static void toggle(){
            if(gripperState == "CLOSED"){
                openGripper();
            }
            else if(gripperState == "OPEN"){
                closeGripper();
            }
        }

        public static void setPosition(double pos){
            GripperServo.setPosition(pos); //auto
        }
    }

    public static class Wrist{

        public static String wristState = "INIT";

        public static void goTop(){
            wristState = "TOP";
            WristServo.setPosition(TopPosition);
        }

        public static void goLowDrop(){
            wristState = "TOP";
            WristServo.setPosition(0.15);
        }

        public static void goInit(){
            wristState = "GRIPPING";
            WristServo.setPosition(InitPosition);
        }

        public static void goGripping(){
            wristState = "GRIPPING";
            WristServo.setPosition(GrippingPosition);
        }

        public static void goAutoTop(){
            wristState = "TOP";
            WristServo.setPosition(TopAutoPosition);
        }

        public static void toggle(){
            if(wristState == "TOP"){
                AlignBar.inside();
                goGripping();
            }
            else if(wristState == "GRIPPING"){
                AlignBar.interMediate();
                goTop();
            }
        }

        public static double getPosition(){
            return WristServo.getPosition();
        }

        public static void setPosition(double pos){
            WristServo.setPosition(pos);
        }


    }

    public static class Slider{

        public static double minPosition = 0.14;
        public static double maxPosition = 0.76;

        public static void moveInside(){
            SliderServo.setPosition(minPosition);
        }

        public static void moveOutside(){
            SliderServo.setPosition(maxPosition);
        }

        public static void moveHalfway(){
            SliderServo.setPosition((maxPosition+minPosition)/2);
        }


        public static void moveSlider(double position){

            position = Range.clip(position, minPosition, maxPosition);
            SliderServo.setPosition(position);
        }

        public static double getPosition(){
            return SliderServo.getPosition();
        }
    }

    public static class AlignBar{

        public static double  outPos = 0.11;
        public static void inside(){
            AlignServo.setPosition(1);
        }

        public static void interMediate(){
            AlignServo.setPosition(0.7);
        }

        public static void outside(){
            AlignServo.setPosition(outPos);
        }

        public static void autoOutside(){AlignServo.setPosition(0.1);}
        public static void autoOutsideHigh(){AlignServo.setPosition(0.25);}

        public static void teleopOut(){
            AlignServo.setPosition(0.16);
        }

        public static double getPosition(){
            return AlignServo.getPosition();
        }

        public static void goodAngle(){
            AlignServo.setPosition(0.61);
        }

        public static void goodAngle2(){AlignServo.setPosition(0.4);}

        public static void goodAngle3(){AlignServo.setPosition(0.2);}

//        public static void pickAlignCone(){
//            AlignServo.setPosition(0.5);
//        }
        public static void dropPosition(){AlignServo.setPosition(0);}

        public static void moveTo(double pos){
            Range.clip(pos, 0.2, 1);
            AlignServo.setPosition(pos);
        }
    }

    public static class AlignBar_2{

        public static double insideForGripping = 0.95;
        public static double insideForTop = 1;
        public static void goInside(){
            AlignServo2.setPosition(1);
        }

        public static void goOutside(){
            AlignServo2.setPosition(0.25);
        }

        public static void setPosition(double pos){
            pos = Range.clip(pos, 0.25, 1);
            AlignServo2.setPosition(pos);
        }
        public static double getPosition(){
            return AlignServo2.getPosition();

        }
    }
}