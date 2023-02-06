package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Sensors {
    private static AnalogInput ultrasoundSensor = null;
    private static RevColorSensorV3 gripperSensor = null;
    private static Rev2mDistanceSensor poleSensor = null;

    private static Telemetry localTelemetry;
    public Sensors(HardwareMap hardwareMap, Telemetry telemetry){
        localTelemetry = telemetry;
        ultrasoundSensor = hardwareMap.get(AnalogInput.class, "ultrasound1");
        gripperSensor = hardwareMap.get(RevColorSensorV3.class, "gripperSensor");
        poleSensor = hardwareMap.get(Rev2mDistanceSensor.class, "poleSensor");

    }


    public static class PoleSensor{
        public static double getDistanceCM(){
            return poleSensor.getDistance(DistanceUnit.CM);
        }
    }


    public static class WallSensor{

        public static double lastDistance = 0;
        public static double getDistanceCM(){
            lastDistance = ultrasoundSensor.getVoltage() * 520.00/3.3;/// 0 to 5v, values given by sensor data sheet
            return lastDistance;
        }

        public static void printDistance(){
            localTelemetry.addData("Wall Distance: ", getDistanceCM());
        }
    }

    public static class GripperSensor{
        public static double getDistanceMM(){
            double dist = gripperSensor.getDistance(DistanceUnit.MM);
            return dist;
        }

        public static double getDistanceINCH(){
            double dist = gripperSensor.getDistance(DistanceUnit.INCH);
            return dist;
        }

        public static void printRGBDistance(){
//            NormalizedRGBA color = gripperSensor.getNormalizedColors();
            localTelemetry.addData("RGB: ", gripperSensor.red() + ", " + gripperSensor.green() + ", " + gripperSensor.blue());
            localTelemetry.addData("Gripper Sensor Distance: ", gripperSensor.getDistance(DistanceUnit.MM));
        }



        public static boolean checkRed(){
            if(gripperSensor.red() > 150){
                localTelemetry.addLine("Red Colour Detected");
            }
            if(gripperSensor.red() > 150){
                return true;
            }
            else{
                return false;
            }
        }

        public static boolean checkBlue(){
            NormalizedRGBA color = gripperSensor.getNormalizedColors(); //red gives good values, here in blue acts randomly, hence for reliability added Normalized colours
            if(gripperSensor.blue() > 150){
                localTelemetry.addLine("Blue Colour Detected");
                return true;
            }
            else{
                return false;
            }
        }

    }
}
