package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.GuardedBy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVars;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
    public static double headingVel;
    public final SampleMecanumDrive drive;
    final Telemetry telemetry;
    final Pose2d startPose = new Pose2d(0, 0, 0);
    final Thread imuThread;
    private final Object imuSemaphore = new Object();
    double drivePowerThrottle=1, drivePowerStrafe=1, drivePowerHeading=1;
    @GuardedBy("imuSemaphore")
    private BNO055IMU imu;
    private double heading;
    public final DoubleSupplier RobotHeadingSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return heading;
        }
    };

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = new SampleMecanumDrive(hardwareMap, telemetry, RobotHeadingSupplier);
        this.telemetry = telemetry;
        drive.setPoseEstimate(startPose);
        imuThread = new Thread(() -> {
            synchronized (imuSemaphore) {
                heading = imu.getAngularOrientation().firstAngle;
                headingVel = imu.getAngularVelocity().zRotationRate;
            }
        });
        synchronized (imuSemaphore) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        drive.update();
        if (GlobalVars.driveTelemetry) {
            telemetry.addData("Heading: ", heading);
        }

    }

    public void startIMUThread() {
        imuThread.start();
    }

    public void slowDownDrive() {
        drivePowerThrottle = 0.4;
        drivePowerStrafe = 0.4;
        drivePowerHeading = 0.4;
    }

    public void normalSpeedDrive() {
        drivePowerThrottle = 1;
        drivePowerStrafe = 1;
        drivePowerHeading = 0.7;
    }

    public void driveWrite(double x, double y, double w) {
        x = x * drivePowerThrottle;
        y = y * drivePowerStrafe;
        w = w * drivePowerHeading;
        drive.setWeightedDrivePower(new Pose2d(x, y, w));
    }

    public void driveWriteFieldOriented(double x, double y, double w) {
        x = x * drivePowerThrottle;
        y = y * drivePowerStrafe;
        w = w * drivePowerHeading;

        Vector2d input = new Vector2d(x * drivePowerThrottle, y * drivePowerStrafe).rotated(-heading);
        drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), w * drivePowerHeading));
    }

    public void killIMUThread() {
        imuThread.interrupt();
    }

    public void resetHeading() {
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }
}
