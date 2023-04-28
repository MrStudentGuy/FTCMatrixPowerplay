package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.toRadians;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class CustomThreeWheel extends ThreeTrackingWheelLocalizer {
    public static double LEFT_ENCODER_MULTIPLIER = 0.9969481180061038*(((0.4379808211246198+0.4313151591980525)/2)*(1.001339436133747 + 1.000333444481494 + 1.001223717877406/3));
    public static double RIGHT_ENCODER_MULTIPLIER = 0.9969481180061038*(((0.4379808211246198+0.4313151591980525)/2)*(1.001339436133747 + 1.000333444481494 + 1.001223717877406/3));
    public static double FRONT_ENCODER_MULTIPLIER = 0.9783938035059111*((1.01633383991929 + 0.9960379822483897 + 0.999000999000999/3)*0.430439522694539);

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.49606/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.23690065; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 1.25; // in; offset of the lateral wheel

    public static double MIN_IMU_UPDATE_INTERVAL = 0.05;
    public static double MIN_STABLE_HEADING_TIME = 0.05;
    public static double HEADING_EPSILON = toRadians(0.5);

    private BNO055IMU imu;
    private double baseExtHeading;

    private ElapsedTime lastIMUUpdateTimer = new ElapsedTime();
    private ElapsedTime stableHeadingTimer = new ElapsedTime();
    private double stableCheckHeading;

    private List<Double> cachedWheelPositions = Collections.emptyList();
    private boolean useCachedWheelPositions = false;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public CustomThreeWheel(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, toRadians(90)) // front
        ));


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        baseExtHeading = getRawExternalHeading();

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double getRawExternalHeading() {
        return Angle.norm(imu.getAngularOrientation().firstAngle);
    }

    private double getExternalHeading() {
        return Angle.norm(getRawExternalHeading() - baseExtHeading);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose) {
        baseExtHeading = Angle.norm(getRawExternalHeading() - pose.getHeading());

        super.setPoseEstimate(pose);
    }

    @Override
    public void update() {
        double currentHeading = getPoseEstimate().getHeading();
        // reset timer and stableCheckHeading if our heading has changed too much
        if (Math.abs(Angle.normDelta(currentHeading - stableCheckHeading)) > HEADING_EPSILON) {
            stableHeadingTimer.reset();
            stableCheckHeading = currentHeading;
        }

        if (lastIMUUpdateTimer.seconds() > MIN_IMU_UPDATE_INTERVAL
                && stableHeadingTimer.seconds() > MIN_STABLE_HEADING_TIME) {
            lastIMUUpdateTimer.reset();

            // load in latest wheel positions and update to apply to pose
            super.update();
            double extHeading = getExternalHeading();
            Pose2d pose = new Pose2d(getPoseEstimate().vec(), extHeading);
            super.setPoseEstimate(pose);

            // Don't update with new positions, but instead use previous (internally cached) wheel positions.
            // This ensures wheel movement isn't "lost" when the lastWheelPositions list (this list is internal to the
            // ThreeTrackingWheelLocalizer/super class) is emptied with our call to setPoseEstimate. Calling update
            // fills lastWheelPositions with the cached wheel positions and allows the later update calls to calculate a
            // movement rather than simply filling the empty lastWheelPositions list and not incrementing the pose.
            useCachedWheelPositions = true;
            super.update();
            useCachedWheelPositions = false;
        } else {
            super.update();
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        if (!useCachedWheelPositions || cachedWheelPositions.isEmpty()) {
            cachedWheelPositions = Arrays.asList(
                    encoderTicksToInches(leftEncoder.getCurrentPosition()) * LEFT_ENCODER_MULTIPLIER,
                    encoderTicksToInches(rightEncoder.getCurrentPosition()) * RIGHT_ENCODER_MULTIPLIER,
                    encoderTicksToInches(frontEncoder.getCurrentPosition()) * FRONT_ENCODER_MULTIPLIER
            );
        }
        return cachedWheelPositions;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * LEFT_ENCODER_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * RIGHT_ENCODER_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * FRONT_ENCODER_MULTIPLIER
        );
    }
}