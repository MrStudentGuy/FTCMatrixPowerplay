package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class ServoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        Servo testServo  = hardwareMap.get(Servo.class, "test");
        waitForStart();


        while(opModeIsActive()){
            testServo.setPosition(Range.clip(gamepad1.left_stick_y, 0.45, 1));
        }
    }
}
