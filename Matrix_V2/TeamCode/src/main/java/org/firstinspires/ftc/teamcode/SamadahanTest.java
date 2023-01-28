package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class SamadahanTest extends LinearOpMode {


    DcMotorEx lm, rm;
    @Override
    public void runOpMode() throws InterruptedException {

        lm = hardwareMap.get(DcMotorEx.class, "lm");
        rm = hardwareMap.get(DcMotorEx.class, "rm");


        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                extendTo(0);  //Lower Most
            }
            else if(gamepad1.b){
                extendTo(3280);  //Top Most
            }
//            lm.setPower(gamepad1.left_stick_y);
//            rm.setPower(gamepad1.left_stick_y);
            telemetry.addData("Positions: ", lm.getCurrentPosition() +", " + rm.getCurrentPosition());
            telemetry.update();

        }
    }

    public void extendTo(int pos){
        lm.setTargetPosition(pos);
        rm.setTargetPosition(pos);
        lm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lm.setPower(1);
        rm.setPower(1);
    }
}
