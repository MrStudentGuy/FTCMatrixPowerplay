package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors;

@TeleOp
public class GripperSensorDebugger extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Sensors sensors = new Sensors(hardwareMap, telemetry);


        waitForStart();

        while(opModeIsActive()){
            Sensors.GripperSensor.printRGBDistance();
            telemetry.update();
        }
    }
}
