package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorHigh;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


@TeleOp
public class Teleop extends CommandOpMode {
    Elevator elevator;
    Turret turret;
    Slider slider;
    Drive drive;
    ElapsedTime timer;

    GamepadEx driverGamepad;
    GamepadEx operatorGamepad;

    Button turretToggleButton, elevatorHighButton, sliderToggleButton;
    @Override
    public void initialize() {
        PhotonCore.enable();
        driverGamepad = new GamepadEx(gamepad1);
        elevator = new Elevator(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        slider = new Slider(this);
        drive = new Drive(hardwareMap, telemetry);

        turretToggleButton = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        elevatorHighButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        sliderToggleButton = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        timer = new ElapsedTime();
        elevatorHighButton.whenPressed(new ElevatorHigh(elevator));
        sliderToggleButton.toggleWhenPressed(slider::goOutside, slider::goInside);
        turretToggleButton.toggleWhenPressed(new TurretCommand(turret, elevator, 180), new TurretCommand(turret, elevator, 0));
    }


    @Override
    public void run() {
        super.run();
        timer.reset();

        if(timer == null){
            drive.normalSpeedDrive();
            timer = new ElapsedTime();
        }

        drive.startIMUThread();
        Pose2d poseEstimate = drive.drive.getPoseEstimate();
        Vector2d input = new Vector2d(-gamepad1.left_stick_y , -gamepad1.left_stick_x ).rotated(-poseEstimate.getHeading());
        drive.drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
        drive.drive.update();//drive is sample mecanum drive


        telemetry.addData("Time: ", timer.seconds());
//        telemetry.addData("Heading: ", drive.RobotHeadingSupplier.getAsDouble());
        telemetry.update();

//        drive.driveWrite(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
    }
}
