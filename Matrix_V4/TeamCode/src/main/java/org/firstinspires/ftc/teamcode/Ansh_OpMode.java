package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.subsystems.Guide;
import org.firstinspires.ftc.teamcode.subsystems.Slider;
import org.firstinspires.ftc.teamcode.subsystems.Sticks;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


abstract public class Ansh_OpMode extends CommandOpMode {

    protected Drive drive;
    protected Turret turret;
    protected Elevator elevator;
    protected EndEffector endEffector;
    protected Guide guide;
    protected Sticks sticks;
    protected Slider slider;

    protected GamepadEx driverGamepad;
    protected GamepadEx operatorGamepad;

    protected void initializeHardware(){
        drive = new Drive(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        slider = new Slider(this);
        endEffector = new EndEffector(hardwareMap, telemetry);
        guide = new Guide(hardwareMap, telemetry);
        sticks = new Sticks(hardwareMap, telemetry);

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }
}
