package org.firstinspires.ftc.teamcode.Tests;

import androidx.lifecycle.Lifecycle;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@Autonomous
//@Disabled
public class TurretPIDTuning extends LinearOpMode {

    Lift lift;
    Turret turret;
    Servos servos;


    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        lift = new Lift(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        servos = new Servos(hardwareMap, telemetry);

        Robot robot = new Robot(hardwareMap, telemetry, lift, turret, servos);



        waitForStart();




        lift.reset();
        turret.reset();

        Servos.Slider.moveInside();
        Servos.AlignBar.inside();
        lift.extendToLowPole();
        Servos.Wrist.goGripping();
        while(opModeIsActive()){
            robot.update();

        }
    }
}
