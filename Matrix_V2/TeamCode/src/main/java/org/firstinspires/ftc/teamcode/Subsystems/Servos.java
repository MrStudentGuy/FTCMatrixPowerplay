package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Servos {
    static Servo GripperServo;
    static Servo WristServo;
    static Servo SliderServo;
    static Servo AlignServo;


    public Servos(HardwareMap hardwareMap, Telemetry telemetry) {
        GripperServo = hardwareMap.get(Servo.class, "Gripper");
        WristServo = hardwareMap.get(Servo.class, "Wrist");
        SliderServo = hardwareMap.get(Servo.class, "Slider");
        AlignServo = hardwareMap.get(Servo.class, "Align"); //auto stop pole shaking, goBilda servo
    }

    public static class Gripper {
        private static final double gripperOpenPosition = 0;
        private static final double gripperClosePosition = 1;
        private static final double gripperBeaconPosition = 0.42;
        public static String gripperState = "OPEN";

        public static void openGripper() {
            gripperState = "OPEN";
            GripperServo.setPosition(gripperOpenPosition);
        }

        public static void closeGripper() {
            gripperState = "CLOSED";
            GripperServo.setPosition(gripperClosePosition);
        }

        public static void gripBeacon() {
            gripperState = "OPEN";
            GripperServo.setPosition(gripperBeaconPosition);
        }

        public static void setPosition(double pos) {
            GripperServo.setPosition(pos); //auto
        }
    }

    public static class Wrist {

        private static final double TopPosition = 0.27;
        private static final double InitPosition = 0.05;
        private static final double GrippingPosition = 0.05;
        private static final double TopAutoPosition = 0.4;
        public static String wristState = "INIT";

        public static void goTop() {
            wristState = "TOP";
            WristServo.setPosition(TopPosition);
        }

        public static void goInit() {
            wristState = "INIT";
            WristServo.setPosition(InitPosition);
        }

        public static void goGripping() {
            wristState = "GRIPPING";
            WristServo.setPosition(GrippingPosition);
        }

        public static void goAutoTop() {
            wristState = "TOP";
            WristServo.setPosition(TopAutoPosition);
        }


    }

    public static class Slider {

        public static void moveInside() {
            SliderServo.setPosition(1);
        }

        public static void moveOutside() {
            SliderServo.setPosition(0.2);
        }

        public static void moveHalfway() {
            SliderServo.setPosition(0.7);
        }


        public static void moveSlider(double position) {

            position = Range.clip(position, 0.2, 1);
            SliderServo.setPosition(position);
        }

        public static double getPosition() {
            return SliderServo.getPosition();
        }
    }

    public static class AlignBar {

        public static double outPos = 0.2;

        public static void inside() {
            AlignServo.setPosition(1);
        }

        public static void interMediate() {
            AlignServo.setPosition(0.9);
        }

        public static void outside() {
            AlignServo.setPosition(outPos);
        }

        public static double getPosition() {
            return AlignServo.getPosition();
        }

        public static void dropPosition() {
            AlignServo.setPosition(0);
        }

        public static void moveTo(double pos) {
            Range.clip(pos, 0.3, 1);
            AlignServo.setPosition(pos);
        }
    }
}
