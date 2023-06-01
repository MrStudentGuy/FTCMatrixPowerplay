package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
@Config
public class liftTune extends LinearOpMode {

    public static double height = 0;
    private double prevHeight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap, telemetry);
        Turret turret = new Turret(hardwareMap, "turret", telemetry);
        Servos servos = new Servos(hardwareMap, telemetry);
        Robot robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
        robot.reset();

        waitForStart();


        while(opModeIsActive()){
            if(height != prevHeight) {
                robot.setTargetHeight(height);
            }
            prevHeight = height;
            robot.update();
        }
    }
}
