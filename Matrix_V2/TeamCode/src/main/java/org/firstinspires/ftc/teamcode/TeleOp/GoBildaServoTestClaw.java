package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Servos;


@TeleOp
public class GoBildaServoTestClaw extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Servos servos = new Servos(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                Servos.Gripper.setPosition(0.25);
            }
            else if(gamepad1.b){
                Servos.Gripper.setPosition(0);
            }
        }




    }
}
