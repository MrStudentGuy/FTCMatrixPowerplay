package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.turret2;

public class teleop2 extends OpMode {


    private final double turretDropPosition = 720;
    turret2 turret = null;
    Lift lift = null;

    @Override
    public void init() {
        turret = new turret2(this);
        lift = new Lift(hardwareMap, telemetry);
    }


    @Override
    public void loop() {

        telemetry.addData("Current: ", turret.getCurrent());
        telemetry.addData("Current: ", lift.getCurrent()[0]);
        if(gamepad1.b) {
            turret.setTurretDegrees(turretDropPosition);
        }
    }
}
