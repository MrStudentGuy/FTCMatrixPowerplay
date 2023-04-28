package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVars;

public class Slider extends SubsystemBase {
    public static double sliderVelocity = 1;
    public static double sliderAcceleration = 1000;
    public static double sliderJerk = 2000;
    private final double outPos = 0.25;
    private final double inPos = 1;
    public double targetSliderPosition = inPos;
    HardwareMap hardwareMap;
    MotionProfile axonProfile;
    Telemetry telemetry;
    ElapsedTime profileTimer;
    private final Servo sliderServo;

    public Slider(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        sliderServo = hardwareMap.get(Servo.class, "Slider");
    }


    @Override
    public void periodic() {
        super.periodic();
        MotionState sliderProfileState = axonProfile.get(profileTimer.seconds());
        setPosition(sliderProfileState.getX());
        if (GlobalVars.sliderTelemetry) {
            telemetry.addData("Slider Servo Position: ", getPosition());
        }
    }

    public double getPosition() {
        return sliderServo.getPosition();
    }

    public void setPosition(double pos) {
        pos = Range.clip(pos, outPos, inPos);
        sliderServo.setPosition(pos);
    }

    public void setTargetSliderPosition(double pos, double vel, double accel, double jerk) {
        double currentServoPosition = getPosition();
        this.targetSliderPosition = pos;
        axonProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(currentServoPosition, 0, 0), new MotionState(this.targetSliderPosition, 0, 0), vel, accel, jerk);
        profileTimer.reset();
    }

    public void setTargetSliderPosition(double pos) {
        this.setTargetSliderPosition(pos, sliderVelocity, sliderAcceleration, sliderJerk);
    }

    public void goOutside() {
        setTargetSliderPosition(outPos);
    }

    public void goInside() {
        setTargetSliderPosition(inPos);
    }
}
