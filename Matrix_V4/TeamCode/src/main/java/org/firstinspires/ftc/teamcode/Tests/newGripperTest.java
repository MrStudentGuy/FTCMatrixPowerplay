package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Disabled
public class newGripperTest extends LinearOpMode {


    Servo gripperServo = null;
    @Override
    public void runOpMode() throws InterruptedException {

        gripperServo = hardwareMap.get(Servo.class, "gripperServo");
        waitForStart();

        while(opModeIsActive()){
            gripperServo.setPosition(gamepad1.left_trigger);
            telemetry.addData("Pos: ",gripperServo.getPosition());
            telemetry.update();
        }
    }
}
