package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Left_Async_V2 extends OpMode {


    final Pose2d droppingPosition0 = new Pose2d(-37.4, -12, Math.toRadians(180));
    final Pose2d droppingPosition = new Pose2d(-37.4, -12.00, Math.toRadians(180));
    final Pose2d pickingPosition = new Pose2d(-49, -12, Math.toRadians(180));


    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Robot robot = null;

    enum State{
        STARTTOCENTER,
        PICK,
        DROP, PARK
    }


    State currentState = State.STARTTOCENTER;

    ElapsedTime timer = new ElapsedTime();


    int counter = 4;
    TrajectorySequence startToCenter;
    Trajectory pick0;
    Trajectory pick;
    Trajectory drop;
    @Override
    public void init() {
        PhotonCore.enable();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);

        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);

        Servos.Gripper.closeGripper();
        Servos.Wrist.goInit();
        Servos.AlignBar.inside();



        Pose2d startPose = new Pose2d(-32, -63.3, Math.toRadians(180));
        robot.setPoseEstimate(startPose);

        startToCenter = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->lift.extendToHighPole())
                .addTemporalMarker(()->{Robot.targetDegree = -140;})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> Servos.Slider.moveSlider(0.55))
                .addTemporalMarker(()-> Servos.Wrist.goGripping())
                .waitSeconds(0.05)
                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.001,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[4], 1);})
                .lineToLinearHeading(pickingPosition)
//                .addTemporalMarker(()->)
                .waitSeconds(0.5)                                      //TODO: POSSIBLE TO REDUCE THIS TIME MAYBE?
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendToHighPole();})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Robot.targetDegree = -140)
                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())


                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[3], 1);})
                .lineToLinearHeading(pickingPosition)
//                .addTemporalMarker(()->)
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendToHighPole();})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Robot.targetDegree = -140)
                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())


                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[2], 1);})
                .lineToLinearHeading(pickingPosition)
//                .addTemporalMarker(()->)
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendToHighPole();})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Robot.targetDegree = -140)
                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())

                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[1], 1);})
                .lineToLinearHeading(pickingPosition)
//                .addTemporalMarker(()->)
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendToHighPole();})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Robot.targetDegree = -140)
                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())

                .addTemporalMarker(()->Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.05,()->{Robot.targetDegree = 0;lift.extendTo(lift.AUTO_POSITION[0], 1);})
                .lineToLinearHeading(pickingPosition)
//                .addTemporalMarker(()->)
                .waitSeconds(0.5)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendToLowPole())
                .addTemporalMarker(()-> Servos.Slider.moveInside())
                .UNSTABLE_addTemporalMarkerOffset(0.01, ()->{lift.extendToHighPole();})
                .lineToLinearHeading(droppingPosition0)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->Robot.targetDegree = -140)
                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Wrist.goTop())
                .addTemporalMarker(()-> Servos.Slider.moveSlider(0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .waitSeconds(0.1)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
//                .addTemporalMarker(()-> )
                .build();

        pick0 = robot.trajectoryBuilder(droppingPosition0)
                .lineToLinearHeading(pickingPosition)
                .build();


        pick = robot.trajectoryBuilder(droppingPosition)
                .lineToLinearHeading(pickingPosition)
                .build();

        drop = robot.trajectoryBuilder(pickingPosition)
                .lineToLinearHeading(droppingPosition)
                .build();

    }

    @Override
    public void start() {
        super.start();
        turret.reset();
        lift.reset();



        timer.reset();
        Servos.Wrist.goTop();
//        Robot.targetHeight = lift.POSITIONS[lift.LOW_POLE];
        robot.followTrajectorySequence(startToCenter);


    }

    boolean preloadLiftFlag = false;
    boolean pickConeFlag = false;
    boolean dropFlag = false;

    @Override
    public void loop() {
        Pose2d poseEstimate = robot.getPoseEstimate();
        robot.update();



    }


}
