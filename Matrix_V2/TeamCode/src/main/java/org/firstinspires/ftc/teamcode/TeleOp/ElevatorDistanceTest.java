package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;


@TeleOp
@Config
public class ElevatorDistanceTest extends LinearOpMode {

    Rev2mDistanceSensor distanceSensor;

    Lift lift = null;
    Servos servos = null;
    public static double distanceTargetMM = 100;
    private final double ticks_in_degree = (10.5 * 28)/360;

    private PIDController controller;
    public static double Kp = 0.004;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "dist");
        servos = new Servos(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        controller = new PIDController(Kp, Ki, Kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servos.Gripper.closeGripper();
        Servos.Slider.moveInside();
        Servos.Wrist.goGripping();
        lift.reset();




        waitForStart();



        while(opModeIsActive()){
            controller.setPID(Kp, Ki, Kd);
            double currentDistance  = distanceSensor.getDistance(DistanceUnit.MM);

            double pid = controller.calculate(currentDistance, distanceTargetMM);

            double ff = Math.cos(Math.toRadians(distanceTargetMM/ticks_in_degree)) * Kf;

            double power = pid + Kf;

            lift.setPower(power);
            Servos.Slider.moveSlider(gamepad1.left_trigger);


            telemetry.addData("Current Distance ", currentDistance);
            telemetry.addData("Target Distance ", distanceTargetMM);
//            telemetry.addData("PIDF: ", Kp + ", " + Ki + ", " + Kd + ", " + Kf);
            telemetry.update();
        }
    }
}
