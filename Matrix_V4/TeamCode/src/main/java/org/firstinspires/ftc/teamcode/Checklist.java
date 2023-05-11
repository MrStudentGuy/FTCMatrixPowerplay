package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Checklist extends LinearOpMode {

    boolean A = false,B= false,Y= false,X= false,DOWN= false,LEFT= false,UP= false,RIGHT= false;
    String guideServoBolt = "(A)Check guide Servo Bolt";
    String stringsTensioned = "(B)Check if the Strings are tensioned";
    String turret0 = "(Y)Turret should be zeroed";
    String lift0 = "(X)Lift should be at the bottom";
    String ConePlaced = "(DOWN)Has the preload cone been placed";
    String Guideinside = "(LEFT)Is the guide in the inside position";
    String BatteryWire = "(UP)Has the battery been secured properly and the wire is inside the chamber";
    String robotPlacement = "(RIGHT)Has the Robot been placed on the field properly";
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();



        while(opModeIsActive()){

            if(gamepad1.a)A = true;
            if(gamepad1.b)B = true;
            if(gamepad1.x)X = true;
            if(gamepad1.y)Y = true;
            if(gamepad1.dpad_up)UP = true;
            if(gamepad1.dpad_down)DOWN = true;
            if(gamepad1.dpad_left)LEFT = true;
            if(gamepad1.dpad_right)RIGHT = true;

            if(!A)telemetry.addLine(guideServoBolt);
            if(!B)telemetry.addLine(stringsTensioned);
            if(!Y)telemetry.addLine(turret0);
            if(!X)telemetry.addLine(lift0);
            if(!DOWN)telemetry.addLine(ConePlaced);
            if(!LEFT)telemetry.addLine(Guideinside);
            if(!UP)telemetry.addLine(BatteryWire);
            if(!RIGHT)telemetry.addLine(robotPlacement);

        }
    }


}
