package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp
@Config
@Disabled
public class LocalizationStuff extends LinearOpMode {
    public  static double leftMulti = 1;
    public static double X_MULTIPLIER = 1.017656780029573 * 0.975609756097561 * 0.9969481180061038; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9783938035059111 * 0.9921414538310413; // Multiplier in the Y direction
    private static final double TICKS_PER_REVOLUTION = 8192;
    private static final double MECANUM_RADIUS = 3.779;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder leftEncoder, rightEncoder, frontEncoder;






    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);



        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while(opModeIsActive()){
            calculateMotorSpeeds(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            double leftEncoderCounts = leftEncoder.getCurrentPosition() * X_MULTIPLIER;
            double rightEncoderCounts = rightEncoder.getCurrentPosition() * X_MULTIPLIER;
            double perpEncoderCounts = frontEncoder.getCurrentPosition() * Y_MULTIPLIER;
            double[] encoders = {leftEncoderCounts, rightEncoderCounts, perpEncoderCounts};
            double[] pose = calculatePose(encoders, 1.49606, 9.9102, 1.25);
            telemetry.addData("Left Encoder: ", leftEncoderCounts * leftMulti);
            telemetry.addData("Right Encoder: ", rightEncoderCounts);
            telemetry.addData("Front Encoder: ", perpEncoderCounts);
            telemetry.addData("X: ", pose[0]);
            telemetry.addData("Y: ", pose[1]);
            telemetry.addData("theta: ", pose[2]);
            telemetry.update();
        }
    }


    public void calculateMotorSpeeds(double forwardBackward, double strafe, double rotation) {
        double frontLeftSpeed = forwardBackward + strafe + rotation;
        double frontRightSpeed = forwardBackward - strafe - rotation;
        double rearLeftSpeed = forwardBackward - strafe + rotation;
        double rearRightSpeed = forwardBackward + strafe - rotation;

        // Normalize the speeds
        double maxSpeed = Math.max(Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)),
                Math.max(Math.abs(rearLeftSpeed), Math.abs(rearRightSpeed)));

        if (maxSpeed > 1.0) {
            frontLeftSpeed /= maxSpeed;
            frontRightSpeed /= maxSpeed;
            rearLeftSpeed /= maxSpeed;
            rearRightSpeed /= maxSpeed;
        }

        // Calculate the actual motor speeds (in RPM or other units based on your motor specifications)
        double frontLeftMotorSpeed =  (frontLeftSpeed * 1);
        double frontRightMotorSpeed =  (frontRightSpeed * 1);
        double rearLeftMotorSpeed =  (rearLeftSpeed * 1);
        double rearRightMotorSpeed =  (rearRightSpeed * 1);

        // Set the motor speeds
        setMotorSpeed(frontLeftMotorSpeed, frontRightMotorSpeed, rearLeftMotorSpeed, rearRightMotorSpeed);
    }

    private void setMotorSpeed(double frontLeft, double frontRight, double rearLeft, double rearRight) {
        leftFront.setPower(frontLeft);
        leftRear.setPower(rearLeft);
        rightFront.setPower(frontRight);
        rightRear.setPower(rearRight);
    }

    public double[] calculatePose(double[] encoders, double wheelDiameter, double wheelDistance, double forwardOffset) {
        double ticksPerInch = TICKS_PER_REVOLUTION / (Math.PI * wheelDiameter);
        double wheelCircumference = Math.PI * wheelDiameter;
        double distancePerTick = wheelCircumference / TICKS_PER_REVOLUTION;

        double leftTicks = encoders[0];
        double rightTicks = encoders[1];
        double perpTicks = encoders[2];

        double x = (leftTicks + rightTicks) * 0.5 * distancePerTick;
        double y = 0;
        if (perpTicks != 0) {
            y = perpTicks * distancePerTick - forwardOffset;
        }

        double theta = (rightTicks - leftTicks) * distancePerTick / wheelDistance;

//        Vector2d pose = new Vector2d(x, y);
//        pose = pose.rotated(theta);

        return new double[]{x, y, Math.toDegrees(theta)};
    }
}
