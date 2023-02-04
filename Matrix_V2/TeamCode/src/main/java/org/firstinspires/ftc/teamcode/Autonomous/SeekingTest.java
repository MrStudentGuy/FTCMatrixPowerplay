package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;


@TeleOp
public class SeekingTest extends LinearOpMode {
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        sensors = new Sensors(hardwareMap, telemetry);

        Servos.Gripper.openGripper();
        Servos.Slider.moveInside();
        Servos.Wrist.goGripping();
        double currentSliderPosition = 0;

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Distance from cone: ", Sensors.GripperSensor.getDistanceMM());
            telemetry.update();
            Servos.Slider.moveSlider(gamepad1.left_trigger);
            if (gamepad1.right_stick_button) {
                Servos.Slider.moveHalfway();
                currentSliderPosition = 0.425;

                timer.reset();
                while (timer.milliseconds() < 2000 && opModeIsActive()) {
                    if (Sensors.GripperSensor.getDistanceMM() < 15) {
                        currentSliderPosition -= 0.001;
                    } else if (Sensors.GripperSensor.getDistanceMM() > 23.5) {
                        currentSliderPosition += 0.01;
                    }
                    else{
                        Servos.Gripper.closeGripper();
                    }

                    Servos.Slider.moveSlider(currentSliderPosition);
                }


            }
        }
    }
}
