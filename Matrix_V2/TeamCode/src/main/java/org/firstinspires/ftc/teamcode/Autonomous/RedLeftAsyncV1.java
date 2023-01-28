package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class RedLeftAsyncV1 extends LinearOpMode {

    //State nomenclature:    CURRENT_TO_TARGET
        enum State{
            START_TO_CENTER,
            CENTER_TO_DROPPRELOAD,
            DROPPC_TO_PICKCONE5,
            DROP5_TO_PICKCONE4,
            DROP4_TO_PICKCONE3,
            DROP3_TO_PICKCONE2,
            DROP2_TO_PICKCONE1,
            PICK5_TO_DROPCONE5,
            PICK4_TO_DROPCONE4,
            DROP3_TO_DROPCONE3,
            DROP2_TO_DROPCONE2,
            DROP1_TO_DROPCONE1,
            DROPCONE1_TO_PARKING
        }

    State currentState = State.START_TO_CENTER;
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;
    Sensors sensors = null;
    final double MAX_SPEED_AUTO = DriveConstants.MAX_VEL;

    /*                  /|\    <----CONE5
                        /|\    <----CONE4
                        /|\    <----CONE3
                        /|\    <----CONE2
                        /|\    <----CONE1
     */






    @Override
    public void runOpMode() throws InterruptedException {

        lift = new Lift(hardwareMap, telemetry);
        servos = new Servos(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        sensors = new Sensors(hardwareMap, telemetry);
        lift.reset();
        turret.reset();
        setInitialPositions();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(-42.75, -63, Math.toRadians(90));
        Pose2d startPoseOverall = new Pose2d(-41.5, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPoseOverall);

        TrajectorySequence startToCenter = drive.trajectorySequenceBuilder(startPoseOverall)
//                .addTemporalMarker(()->Servos.Wrist.goGripping())
//                .addTemporalMarker(()->lift.extendToLowPole())
                .lineToConstantHeading(new Vector2d(-36, -63))
//                .UNSTABLE_addTemporalMarkerOffset(1, ()->turret.setDegree(-45))
                .lineToConstantHeading(new Vector2d(-36, -12))
//                .UNSTABLE_addTemporalMarkerOffset(-1, ()->lift.extendToHighPole())
//                .addTemporalMarker(()->Servos.Slider.moveOutside())
//                .waitSeconds(1)
//                .addTemporalMarker(()->Servos.Gripper.openGripper())
                .build();


        waitForStart();
        currentState = State.START_TO_CENTER;
        drive.followTrajectorySequenceAsync(startToCenter);

        while(opModeIsActive()){
            drive.update();
            switch (currentState){
                case START_TO_CENTER:
                    if(!drive.isBusy()){
                        currentState = State.CENTER_TO_DROPPRELOAD;
//                        drive.followTrajectorySequenceAsync();
                    }
                    break;
                case CENTER_TO_DROPPRELOAD:
                    break;
//                    if(!)
            }


        }
    }


    public void buildTrajectories(){

    }


    private void setInitialPositions(){
        lift.extendTo(0, 1);
        Servos.Gripper.closeGripper();
        sleep(30);
        Servos.Wrist.goInit();
        turret.setDegree(0);
    }



}
