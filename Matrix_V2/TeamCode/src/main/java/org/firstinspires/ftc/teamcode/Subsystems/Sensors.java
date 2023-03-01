package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem Class containing all Sensors used on Team Matrix's PowerPlay Robot
 */
public class Sensors {
    @Deprecated
    private static AnalogInput ultrasoundSensor = null;

    private static RevColorSensorV3 gripperSensor = null;
    @Deprecated
    private static Rev2mDistanceSensor poleSensor = null;

    private static Telemetry localTelemetry;

    /**
     * Create a new Subsystem of Sensors.
     * @param hardwareMap Pass in the hardwaremap used by the opMode
     * @param telemetry Pass in telemetry to display values
     */
    public Sensors(HardwareMap hardwareMap, Telemetry telemetry){
        localTelemetry = telemetry;
        ultrasoundSensor = hardwareMap.get(AnalogInput.class, "ultrasound1");
//        gripperSensor = hardwareMap.get(RevColorSensorV3.class, "gripperSensor");
//        poleSensor = hardwareMap.get(Rev2mDistanceSensor.class, "poleSensor");

    }


    @Deprecated
    public static class PoleSensor{
        public static double getDistanceCM(){
            return poleSensor.getDistance(DistanceUnit.CM);
        }
    }

@Deprecated
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

    /**
     * Sensor mounted near the claw to detect the presence of cones. The wire for this sensor is currently not working and needs replacement.
     */
    public static class GripperSensor{

        /**
         * Get the distance from cone in Millimeters
         * @return The distance of the cone in mm
         */
        public static double getDistanceMM(){
            double dist = gripperSensor.getDistance(DistanceUnit.MM);
            return dist;
        }
        /**
         * Get the distance from cone in inches
         * @return The distance of the cone in inches
         */
        public static double getDistanceINCH(){
            double dist = gripperSensor.getDistance(DistanceUnit.INCH);
            return dist;
        }


        /**
         * Print the RGB and the distance to telemetry
         */
        public static void printRGBDistance(){
//            NormalizedRGBA color = gripperSensor.getNormalizedColors();
            localTelemetry.addData("RGB: ", gripperSensor.red() + ", " + gripperSensor.green() + ", " + gripperSensor.blue());
            localTelemetry.addData("Gripper Sensor Distance: ", gripperSensor.getDistance(DistanceUnit.MM));
        }


        /**
         * Check for prescence of red colour in front of the sensor
         * @return True if red is detected
         */
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

        /**
         * Check for prescence of blue colour in front of the sensor
         * @return True if blue is detected
         */
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
