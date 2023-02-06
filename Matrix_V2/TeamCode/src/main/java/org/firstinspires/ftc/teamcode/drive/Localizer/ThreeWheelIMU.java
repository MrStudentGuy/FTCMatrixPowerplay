package org.firstinspires.ftc.teamcode.drive.Localizer;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Localizer.Localizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class ThreeWheelIMU implements Localizer {
    //https://gm0.org/en/latest/_images/offsets-and-trackwidth.png
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.49606 / 2; // in
    public static double LATERAL_DISTANCE = 10.23690065; // in; distance between the left and right wheels

    public static double leftEncoderInches = 0, rightEncoderInches = 0, frontEncoderInches = 0;
    public static double PrevLeftEncoderInches = 0, PrevRightEncoderInches = 0, PrevFrontEncoderInches = 0;
    public static double DeltaLeftEncoderInches = 0, DeltaRightEncoderInches = 0, DeltaFrontEncoderInches = 0;

    public static double FORWARD_OFFSET = 1.25; // in; offset of the lateral wheel

    public Pose2d poseEstimate, pastPoseEstimate;
    SampleMecanumDrive drive;
    HardwareMap hardwareMap;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public ThreeWheelIMU(HardwareMap localHardwareMap, SampleMecanumDrive localDrive) {

        this.drive = localDrive;
        this.hardwareMap = localHardwareMap;
        poseEstimate = new Pose2d();
        pastPoseEstimate = new Pose2d();

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    @Override
    public void periodic() {
        updateWheelPositions();
        double deltaTheta = DeltaLeftEncoderInches - DeltaRightEncoderInches/LATERAL_DISTANCE;
        double deltaX = DeltaLeftEncoderInches + DeltaRightEncoderInches/2;
        double deltaY = DeltaFrontEncoderInches - (FORWARD_OFFSET * deltaTheta);
        double thetaO = poseEstimate.getHeading();

        double[][] A = {{cos(thetaO) , -(sin(thetaO)), 0}, {sin(thetaO) , cos(thetaO), 0}, {0 , 0, 1}};
        double[][] B = {{sin(deltaTheta)/deltaTheta, (cos(deltaTheta)-1)/deltaTheta, 0}, {(1-cos(deltaTheta))/deltaTheta, sin(deltaTheta)/deltaTheta, 0}, {0, 0, 1}};
        double[][] C = {{deltaX, deltaY, deltaTheta}};

        double AB[][] = matrixMultiplication(A, 3, 3, B, 3, 3);
        double ABC[][] = matrixMultiplication(AB, 3, 3, C, 1, 3);

        pastPoseEstimate = poseEstimate;
        poseEstimate = new Pose2d(poseEstimate.getX() + ABC[0][0], poseEstimate.getY() + ABC[0][1] + poseEstimate.getHeading() + ABC[0][3]);


        //https://file.tavsys.net/control/controls-engineering-in-frc.pdf



        Pose2d deltaPose = new Pose2d(poseEstimate.getX() - pastPoseEstimate.getX(),
                poseEstimate.getY() - pastPoseEstimate.getY(),
                poseEstimate.getHeading() - pastPoseEstimate.getHeading());

        pastPoseEstimate = poseEstimate;
    }

    public void updateWheelPositions(){
        leftEncoderInches = encoderTicksToInches(leftEncoder.getCurrentPosition());
        rightEncoderInches = encoderTicksToInches(rightEncoder.getCurrentPosition());
        frontEncoderInches = encoderTicksToInches(frontEncoder.getCurrentPosition());

        DeltaLeftEncoderInches = leftEncoderInches - PrevLeftEncoderInches;
        DeltaRightEncoderInches = rightEncoderInches - PrevRightEncoderInches;
        DeltaFrontEncoderInches = frontEncoderInches - PrevFrontEncoderInches;

        PrevLeftEncoderInches = leftEncoderInches;
        PrevRightEncoderInches = rightEncoderInches;
        PrevFrontEncoderInches = frontEncoderInches;
    }

    @Override
    public Pose2d getPose() {
        return poseEstimate;
    }

    @Override
    public void setPose(Pose2d pose) {
        poseEstimate = pose;
    }


    /**
     * to find out matrix multiplication
     *
     * @param matrix1 First matrix
     * @param rows1   Number of rows in matrix 1
     * @param cols1   Number of columns in matrix 1
     * @param matrix2 Second matrix
     * @param rows2   Number of rows in matrix 2
     * @param cols2   Number of columns in matrix 2
     * @return the result matrix (matrix 1 and matrix 2
     * multiplication)
     */
    public static double[][] matrixMultiplication(
            double[][] matrix1, int rows1, int cols1,
            double[][] matrix2, int rows2, int cols2)
    {

        // Required condition for matrix multiplication
//        if (cols1 != rows2) {
//            throw new Exception("Invalid matrix given.");
//        }

        // create a result matrix
        double resultMatrix[][] = new double[rows1][cols2];

        // Core logic for 2 matrices multiplication
        for (int i = 0; i < resultMatrix.length; i++)
        {
            for (int j = 0;
                 j < resultMatrix[i].length;
                 j++)
            {
                for (int k = 0; k < cols1; k++)
                {
                    resultMatrix[i][j]
                            += matrix1[i][k] * matrix2[k][j];
                }
            }
        }
        return resultMatrix;
    }
}