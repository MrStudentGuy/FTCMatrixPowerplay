package org.firstinspires.ftc.teamcode.Autonomous.DeprecatedAutos;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.TransferClass;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
@Disabled
public class MatrixAutoAmazing extends LinearOpMode {

    final Pose2d droppingPosition0 = new Pose2d(-42, -12, Math.toRadians(180));       // Positions on the field to drop cone into pole
    final Pose2d droppingPosition = new Pose2d(-42.01, -12, Math.toRadians(180));
    final Pose2d pickingPosition = new Pose2d(-42, -12, Math.toRadians(180));
    final Pose2d preloadDropPosition = new Pose2d(-42, -12, Math.toRadians(180));
    final Pose2d preloadDropPosition0 = new Pose2d(-44.5, -12, Math.toRadians(180));

    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);

        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        turret.reset();
        lift.reset();

        Servos.Slider.moveInside();
        Servos.Gripper.openGripper();
        Servos.Wrist.goGripping();

        Pose2d startPose = new Pose2d(-31.8, -63.3, Math.toRadians(180));
        robot.setPoseEstimate(startPose);

        TransferClass.offsetpose = 90;

        TrajectorySequence autonomousTrajectory = robot.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                        .addTemporalMarker(0.3,()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(0.8, ()->{Robot.targetDegree = -147;
                    Servos.Wrist.goTop();
                    Servos.AlignBar.outside();
                })
                .lineToLinearHeading(preloadDropPosition)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.3)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
                .waitSeconds(0.2)
                .addTemporalMarker( ()-> {dropCone();})
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .waitSeconds(0.3)
                .addTemporalMarker( ()->{
                    Servos.AlignBar.inside();
                    Servos.Gripper.closeGripper();})
                .addTemporalMarker( ()->{Robot.targetDegree = 0;})
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[4], 1))
                .lineToLinearHeading(preloadDropPosition0)

                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->{Robot.targetDegree = -151;
                    Servos.Wrist.goTop();
                    Servos.AlignBar.outside();
                })
                .addTemporalMarker(()-> Servos.Slider.moveInside())

                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {dropCone();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .waitSeconds(0.3)
                .addTemporalMarker( ()->{
                    Servos.AlignBar.inside();
                    Servos.Gripper.closeGripper();})
                .addTemporalMarker( ()->{Robot.targetDegree = 0;})
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[3], 1))


                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->{Robot.targetDegree = -151;
                    Servos.Wrist.goTop();
                    Servos.AlignBar.outside();
                })
                .addTemporalMarker(()-> Servos.Slider.moveInside())

                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {dropCone();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .waitSeconds(0.3)
                .addTemporalMarker( ()->{
                    Servos.AlignBar.inside();
                    Servos.Gripper.closeGripper();})
                .addTemporalMarker( ()->{Robot.targetDegree = 0;})
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[2], 1))


                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->{Robot.targetDegree = -151;
                    Servos.Wrist.goTop();
                    Servos.AlignBar.outside();
                })
                .addTemporalMarker(()-> Servos.Slider.moveInside())

                .waitSeconds(1)

                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {dropCone();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .waitSeconds(0.3)
                .addTemporalMarker( ()->{
                    Servos.AlignBar.inside();
                    Servos.Gripper.closeGripper();})
                .addTemporalMarker( ()->{Robot.targetDegree = 0;})
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[1], 1))


                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->{Robot.targetDegree = -151;
                    Servos.Wrist.goTop();
                    Servos.AlignBar.outside();
                })
                .addTemporalMarker(()-> Servos.Slider.moveInside())

                .waitSeconds(1)


                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {dropCone();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .waitSeconds(0.3)
                .addTemporalMarker( ()->{
                    Servos.AlignBar.inside();
                    Servos.Gripper.closeGripper();})
                .addTemporalMarker( ()->{Robot.targetDegree = 0;})
                .addTemporalMarker(()->lift.extendTo(lift.AUTO_POSITION[0], 1))



                .waitSeconds(0.8)
                .addTemporalMarker(()-> Servos.Gripper.openGripper())
                .waitSeconds(0.4)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.2)
                .addTemporalMarker(()-> Servos.Gripper.closeGripper())
                .waitSeconds(0.3)
                .addTemporalMarker(()->lift.extendTo(lift.POSITIONS[lift.HIGH_POLE], 1))
                .addTemporalMarker(()->{Robot.targetDegree = -151;
                    Servos.Wrist.goTop();
                    Servos.AlignBar.outside();
                })
                .addTemporalMarker(()-> Servos.Slider.moveInside())

                .waitSeconds(1)
                .addTemporalMarker(()-> Servos.Slider.moveOutside())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Servos.Wrist.goGripping())
                .addTemporalMarker(()-> Servos.AlignBar.dropPosition())
                .waitSeconds(0.5)
                .addTemporalMarker( ()-> {dropCone();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->Servos.Slider.moveInside())
                .addTemporalMarker(()-> Servos.AlignBar.outside())
                .waitSeconds(0.3)
                .addTemporalMarker( ()->{
                    Servos.AlignBar.inside();
                    Servos.Gripper.closeGripper();})
                .addTemporalMarker( ()->{Robot.targetDegree = 0;})



                .waitSeconds(10)


                        .build();



        waitForStart();
        robot.followTrajectorySequence(autonomousTrajectory);



    }


    void dropCone(){
        Servos.Gripper.openGripper();

    }
}
