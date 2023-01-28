package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class ultrasoundTest extends LinearOpMode {

    AnalogInput sensor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(AnalogInput.class, "ultrasound1");


        waitForStart();


        while(opModeIsActive()){
            double V = sensor.getVoltage();
            double d = V * 520.00/3.3;
            telemetry.addData("Value: ", V);
            telemetry.addData("Distance: ", d);
            telemetry.update();
        }
    }
}
