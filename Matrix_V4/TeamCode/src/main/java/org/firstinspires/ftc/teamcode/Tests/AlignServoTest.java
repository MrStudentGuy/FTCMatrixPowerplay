package org.firstinspires.ftc.teamcode.Tests;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@TeleOp
public class AlignServoTest extends LinearOpMode {
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        sensors = new Sensors(hardwareMap, telemetry);


        lift.reset();
        turret.reset();
        setInitialPositions();



        waitForStart();


        while(opModeIsActive()){



            boolean A = gamepad1.a;                  //x
            boolean B = gamepad1.b;                  //o
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
            boolean R3 = gamepad1.right_stick_button;
            boolean L3 = gamepad1.left_stick_button;

            if (UP) {
                Servos.Wrist.goTop();
                lift.extendToHighPole();
            } else if (RIGHT) {
                Servos.Wrist.goGripping();
                lift.extendToGrippingPosition();
            } else if (DOWN) {
                Servos.Wrist.goTop();
                lift.extendToLowPole();
            } else if (LEFT) {
                Servos.Wrist.goTop();
                lift.extendToMidPole();
            }

            if(gamepad1.a){
                alignmentOut();
            }
            else if(gamepad1.b){
                alignmentIn();
            }

            if(gamepad1.dpad_down){
                Servos.Slider.moveInside();
            }

        }
    }

    void alignmentOut(){
        if(Servos.Slider.getPosition() < 1) {
            Servos.Slider.moveOutside();
            sleep(1000);
        }
        if(lift.getPosition()[0] >= lift.POSITIONS[lift.LOW_POLE]){
            Servos.AlignBar.outside();
        }
    }

    void alignmentIn(){
        if(Servos.Slider.getPosition() < 1) {
            Servos.Slider.moveOutside();
            sleep(1000);
        }
        if(lift.getPosition()[0] >= lift.POSITIONS[lift.LOW_POLE]){
            Servos.AlignBar.inside();
        }
    }



    private void setInitialPositions(){
        lift.extendTo(0, 1);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.AlignBar.inside();
        Servos.Wrist.goInit();
        turret.setDegree(0);
    }
}
