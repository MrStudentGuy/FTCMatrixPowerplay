package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@TeleOp
public class V4 extends LinearOpMode {
    ElapsedTime timerForSafe;
    boolean flagForSafe;


    ElapsedTime timerForLowDrop;
    ElapsedTime timerForThodaUpar;
    ElapsedTime returnToGrippingAfterDropTimer;
    boolean flagForLowDrop = false, returnToGrippingAfterDropFlag = false, thodaUparFlag = false;
    boolean vibrateFlag = false;
    double speedThrottle = 1, headingThrottle = 1;
    Lift lift = null;
    Servos servos = null;
    Turret turret = null;

    Robot robot = null;

    ElapsedTime teleopTimer = null;


    boolean fallenConeFlag = false, turretFlag = false;
    private final int A = 0;
    private final int B = 1;
    private final int Y = 2;
    private final int X = 3;
    private final int DOWN = 4;
    private final int RIGHT = 5;
    private final int UP = 6;
    private final int LEFT = 7;
    private final int RB = 8;
    private final int LB = 9;
    private final int RS = 10;
    private final int LS = 11;
    private final int START = 12;
    private final int BACK = 13;

    private final int LY = 0, LX = 1, RY = 2, RX = 3, LT = 4, RT = 5;
    private final int R2 = 14;
    private final int L2 = 15;
    private boolean[] operatorButtons, driverButtons;
    private boolean[] previousDriverButtons;
    private boolean[] previousOperatorButtons;

    private double[] previousDriverTriggers, previousOperatorTriggers;


    private double[] driverTriggers, operatorTriggers;
    private boolean shiftMode1 = false;
    private boolean shiftMode2 = false;

    private int selection = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        timerForSafe = new ElapsedTime();
        timerForLowDrop = new ElapsedTime();
        timerForThodaUpar = new ElapsedTime();
        returnToGrippingAfterDropTimer = new ElapsedTime();
        lift = new Lift(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, "turret", telemetry);
        servos = new Servos(hardwareMap, telemetry);
        robot = new Robot(hardwareMap, telemetry, lift, turret, servos);
        Servos.Slider.minPosition = 0.25;


//        robot.reset();

        Robot.targetDegree = 0;
        Servos.Wrist.goGripping();
        Servos.Gripper.closeGripper();
        Servos.AlignBar.inside();
        Servos.Slider.moveInside();
        Servos.AlignBar_2.goInside();
        robot.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));





        waitForStart();




        while(opModeIsActive()){
            driverButtons = readDriverButtons();
            operatorButtons = readOperatorButtons();
            driverTriggers = readDriverTriggers();
            operatorTriggers = readOperatorTriggers();

            if(previousDriverButtons == null){
                previousDriverButtons = driverButtons;
            }
            if(previousOperatorButtons == null){
                previousOperatorButtons = operatorButtons;
            }


            shiftMode1 = operatorButtons[L2];

            drive(-driverTriggers[LY], -driverTriggers[LX], -driverTriggers[RX]);


            if (shiftMode1) {
                shiftFunctionality();
            }
            else{
                normalFunctionality();
            }

            common();

//            telemetry.addData("RT: ", driverButtons[RB] && !previousDriverButtons[RB]);
            previousDriverButtons = driverButtons;
            previousOperatorButtons = operatorButtons;
            previousDriverTriggers = driverTriggers;
            previousOperatorTriggers = operatorTriggers;


            telemetry.addData("Lift Left Current: ", lift.getCurrent()[0]);
            telemetry.addData("Lift Right Current: ", lift.getCurrent()[1]);
            telemetry.addData("Trigger: ", driverTriggers[LT]);
            telemetry.addData("Slider: ", Servos.Slider.getPosition());
            telemetry.addData("Selection: ", selection);
//            telemetry.addData("Turret Degrees: ", turret.getDegree());
            telemetry.addData("Lift Height: ", lift.getPosition()[0]);
//            telemetry.update();
        }
    }


    private void common(){


        if(driverButtons[X]){
            robot.setPoseEstimate(new Pose2d());
        }


        if(vibrateFlag){
            vibrateFlag = false;
            gamepad1.rumbleBlips(2);
        }
        else{

        }


        if(driverButtons[DOWN]){
            selection = lift.POSITIONS[lift.GRIPPING_POSITION];
            lift.extendTo(selection, 1);
        }

        if(driverButtons[LB] && lift.getPosition()[0]<lift.POSITIONS[lift.LOW_POLE]+50 && lift.getPosition()[0]>lift.POSITIONS[lift.LOW_POLE]-50){
            Servos.Wrist.goLowDrop();
            timerForLowDrop.reset();
            flagForLowDrop = true;
        }

        if(flagForLowDrop && timerForLowDrop.milliseconds()>400){
            flagForLowDrop = false;
            Servos.Wrist.goGripping();
        }


        robot.update();

        if(operatorButtons[R2]){
            speedThrottle = 0.38;
            headingThrottle = 0.5;
        }
        else{
            speedThrottle = 1;
            headingThrottle = 1;
        }

        if(driverButtons[B] && !previousDriverButtons[B]){
            if(fallenConeFlag){
//                Servos.AlignBar_2.goOutside();
                fallenConeFlag = false;
            }
            else{
                Servos.AlignBar_2.goInside();
                fallenConeFlag = true;
//                Servos.Gripper.openGripper();
//                Servos.Wrist.goGripping();
            }
        }
        if(driverButtons[LB] && !previousDriverButtons[LB]){
            Servos.Wrist.toggle();
            if(Servos.Wrist.wristState == "GRIPPING"){
                if(lift.getPosition()[0] < lift.POSITIONS[lift.LOW_POLE]-20){
                    timerForSafe.reset();
                    flagForSafe = true;
                }
                Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForGripping);
            }
            else if(Servos.Wrist.wristState == "TOP"){
                if(lift.getPosition()[0] < lift.POSITIONS[lift.LOW_POLE]-20){
                    timerForSafe.reset();
                    flagForSafe = true;
                }
                Servos.Slider.moveSlider(0.3);
                Servos.AlignBar_2.setPosition(Servos.AlignBar_2.insideForTop);
            }
        }

        if(flagForSafe && timerForSafe.milliseconds() < 400){
            Servos.Slider.moveSlider(0.35);
        }
        else if(timerForSafe.milliseconds() >= 400){
            flagForSafe = false;
            Servos.Slider.moveSlider(gamepad1.left_trigger);
        }

        if(driverButtons[RB] && !previousDriverButtons[RB]){
            if(Servos.Gripper.gripperState == "CLOSED" && lift.getPosition()[0] > lift.POSITIONS[lift.GRIPPING_POSITION]+300){
                returnToGrippingAfterDropTimer.reset();
                returnToGrippingAfterDropFlag = true;
            }
            if(Servos.Gripper.gripperState == "OPEN" && lift.getPosition()[0] < lift.POSITIONS[lift.LOW_POLE]-100){
                timerForThodaUpar.reset();
                thodaUparFlag = true;
            }
            Servos.Gripper.toggle();
        }

        if(thodaUparFlag && timerForThodaUpar.milliseconds()>300){
            lift.extendTo(lift.POSITIONS[lift.GRIPPING_POSITION]+80, 1);
            thodaUparFlag = false;
        }

        if(returnToGrippingAfterDropFlag && returnToGrippingAfterDropTimer.milliseconds() > 1000){
            Robot.targetDegree = 0;
            Servos.Gripper.closeGripper();
            returnToGrippingAfterDropFlag = false;
            Servos.Wrist.goGripping();
            lift.extendTo(lift.POSITIONS[lift.GRIPPING_POSITION], 1);
        }

        if(operatorButtons[UP]){
            vibrateFlag = true;
            selection+=10;
        }
        else if(operatorButtons[DOWN]){
            vibrateFlag = true;
            selection-=10;
        }

        if(driverButtons[R2] && !previousDriverButtons[R2] && selection>=lift.POSITIONS[lift.MID_POLE]){
            Servos.Wrist.goTop();
        }
        if(driverButtons[R2]){
            if(selection<lift.POSITIONS[lift.LOW_POLE]-40){
                Servos.Wrist.goGripping();
                Servos.Gripper.closeGripper();
            }
            lift.extendTo(selection, 1);
        }
        turretSafety();
    }



    private void shiftFunctionality(){

        telemetry.addLine("Shift triggered");
        if(operatorButtons[X]){
            vibrateFlag = true;
            selection = lift.AUTO_POSITION[1];
        }
        else if(operatorButtons[Y]){
            vibrateFlag = true;
            selection = lift.AUTO_POSITION[2];
        }
        else if(operatorButtons[B]){
            vibrateFlag = true;
            selection = lift.AUTO_POSITION[3];
        }
        else if(operatorButtons[A]){
            vibrateFlag = true;
            selection = lift.AUTO_POSITION[4];
        }


        if(operatorButtons[LEFT]){
            turretFlag = true;
            Robot.targetDegree = 90;
        }
        else if(operatorButtons[RIGHT]){
            turretFlag = true;
            Robot.targetDegree = -90;
        }
        else if(operatorButtons[LB] && !previousOperatorButtons[LB]){
            if(turretFlag){
                turretFlag = false;
                Robot.targetDegree = 0;
            }
            else {
                turretFlag = true;
                Robot.targetDegree = 180;
            }
        }
    }



    private void normalFunctionality(){

        telemetry.addLine("Normal triggered");
        if(operatorButtons[X]){
            vibrateFlag = true;
            selection = lift.POSITIONS[lift.GRIPPING_POSITION];
        }
        else if(operatorButtons[Y]){
            vibrateFlag = true;
            selection = lift.POSITIONS[lift.LOW_POLE];
        }
        else if(operatorButtons[B]){
            vibrateFlag = true;
            selection = lift.POSITIONS[lift.MID_POLE];
        }
        else if(operatorButtons[A]){
            vibrateFlag = true;
            selection = lift.POSITIONS[lift.HIGH_POLE] - 50;
        }

        if(operatorButtons[LEFT]){
            turretFlag = true;
            Robot.targetDegree = 135;
        }
        else if(operatorButtons[RIGHT]){
            turretFlag = true;
            Robot.targetDegree = -135;
        }
        else if(operatorButtons[LB] && !previousOperatorButtons[LB]){
            if(turretFlag){
                turretFlag = false;
                Robot.targetDegree = 0;
            }
            else {
                turretFlag = true;
                Robot.targetDegree = 180;
            }
        }
    }

    private void turretSafety(){
        if(Math.abs(turret.getDegree())> 172  && lift.getPosition()[0]<lift.POSITIONS[lift.LOW_POLE]-50){
            Servos.Slider.minPosition = 0.25;
            if(Servos.Slider.getPosition() < 0.25){
                Servos.Slider.moveSlider(0.25);
            }
        }
        else if(Math.abs(turret.getDegree()) > 114  && lift.getPosition()[0]<lift.POSITIONS[lift.LOW_POLE]-50){
            Servos.Slider.minPosition = 0.4;
            if(Servos.Slider.getPosition() < 0.4){
                Servos.Slider.moveSlider(0.4);
            }
        }
        else if(Math.abs(turret.getDegree()) > 66  && lift.getPosition()[0]<lift.POSITIONS[lift.LOW_POLE]-50){
            Servos.Slider.minPosition = 0.25;
            if(Servos.Slider.getPosition() < 0.25){
                Servos.Slider.moveSlider(0.25);
            }
        }
        else if(Math.abs(turret.getDegree()) > 20 && lift.getPosition()[0]<lift.POSITIONS[lift.LOW_POLE]-50){
            Servos.Slider.minPosition = 0.4;
            if(Servos.Slider.getPosition() < 0.4){
                Servos.Slider.moveSlider(0.4);
            }
        }
        else {
            Servos.Slider.minPosition = 0;
        }
    }


    private boolean[] readDriverButtons(){
        return readGamepadButton(gamepad1);
    }

    private void drive(double y, double x, double w){
        double heading = robot.getPoseEstimate().getHeading();
        Vector2d input = new Vector2d(y,x).rotated(-heading);
        Pose2d drivePowers = new Pose2d(input.getX()*speedThrottle, input.getY()*speedThrottle, w*headingThrottle);
        robot.setWeightedDrivePower(drivePowers);
    }

    private double[] readGamepadTriggers(Gamepad gamepad){
        double LY = gamepad.left_stick_y;
        double LX = gamepad.left_stick_x;
        double RY = gamepad.right_stick_y;
        double RX = gamepad.right_stick_x;
        double LT = gamepad.left_trigger;
        double RT = gamepad.right_trigger;

        return new double[] {LY, LX, RY, RX, LT, RT};
    }

    private double[] readDriverTriggers(){
        return readGamepadTriggers(gamepad1);
    }

    private double[] readOperatorTriggers(){
        return readGamepadTriggers(gamepad2);
    }

    private boolean[] readGamepadButton(Gamepad gamepad){
        boolean A = gamepad.a;
        boolean B = gamepad.b;
        boolean X = gamepad.x;
        boolean Y = gamepad.y;
        boolean RB = gamepad.right_bumper;
        boolean LB = gamepad.left_bumper;
        boolean UP = gamepad.dpad_up;
        boolean DOWN = gamepad.dpad_down;
        boolean LEFT = gamepad.dpad_left;
        boolean RIGHT = gamepad.dpad_right;
        boolean RS = gamepad.right_stick_button;
        boolean LS = gamepad.left_stick_button;
        boolean START = gamepad.start;
        boolean BACK = gamepad.back;
        boolean R2 = gamepad.right_trigger>0.8 ? true:false;
        boolean L2 = gamepad.left_trigger>0.8 ? true:false;

        boolean[] pressedButtons = new boolean[] {A, B, Y, X, DOWN, RIGHT, UP, LEFT, RB, LB, RS, LS, START, BACK, R2, L2};

        return pressedButtons;
    }

    private boolean[] readOperatorButtons(){
        return readGamepadButton(gamepad2);
    }
}
