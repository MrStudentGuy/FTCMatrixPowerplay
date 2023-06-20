package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.util.ServoMotionController;

@TeleOp
@Config
@Disabled
public class ServoMotionControllerTest extends LinearOpMode {

    public static double sliderMaxVelocity = 20;
    public static double sliderMaxAcceleration = 2.8;
    Servo SliderServo, alignServo, wristServo;
    MotionProfile sliderProfile;
    ElapsedTime sliderProfileTimer;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        SliderServo = hardwareMap.get(Servo.class, "Slider");
        wristServo = hardwareMap.get(Servo.class, "Wrist");
        SliderServo.setDirection(Servo.Direction.REVERSE);
        alignServo = hardwareMap.get(Servo.class, "Align");
        sliderProfileTimer = new ElapsedTime();
        alignServo.setPosition(1);

        waitForStart();

        wristServo.setPosition(0.2578);
        alignServo.setPosition(0.287);

        while(opModeIsActive()){
            if(sliderProfile != null){
                MotionState sliderState = sliderProfile.get(sliderProfileTimer.seconds());
                setPosition(sliderState.getX());
                telemetry.addData("Slider X: ", sliderProfile.get(sliderProfileTimer.seconds()).getX());
                telemetry.update();
            }
            if(gamepad1.a){
                setTarget(0.8,sliderMaxVelocity, sliderMaxAcceleration);
            }
            else if(gamepad1.b){
                setTarget(0, sliderMaxVelocity, sliderMaxAcceleration);
            }

//            telemetry.addData("")
        }
    }


    private void setPosition(double pos){
        pos = Range.clip(pos, 0, 0.8);
        SliderServo.setPosition(pos);
    }

    private void setTarget(double pos, double maxVel, double maxAccel){
        sliderProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(SliderServo.getPosition(), 0, 0),
                new MotionState(pos, 0, 0),
                maxVel,
                maxAccel
        );
        sliderProfileTimer.reset();
    }
}
