package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.outoftheboxrobotics.photoncore.PhotonCore;

import org.firstinspires.ftc.teamcode.GlobalVars;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.Subsystems.Guide;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Sticks;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class IntoTheMatrix extends CommandOpMode {

    //----------------------------------------------------------------------
    EndEffector endEffector;
    Guide guide;
    Elevator elevator;
    Turret turret;
    Sticks sticks;
    Drive drive;
    Slider slider;
    //----------------------------------------------------------------------

    GamepadEx driver;
    GamepadEx operator;

    Button grippingButton, wristButton;
    TriggerReader slowModeButton;

    @Override
    public void initialize() {
        PhotonCore.enable();
        GlobalVars.currentOpModeType = GlobalVars.runMode.TELEOP;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        this.endEffector = new EndEffector(hardwareMap, telemetry);
        this.guide = new Guide(hardwareMap, telemetry);
        this.turret = new Turret(hardwareMap, telemetry);
        this.elevator = new Elevator(hardwareMap, telemetry);
        this.drive = new Drive(hardwareMap, telemetry, this);
        this.slider = new Slider(hardwareMap, telemetry, this);

        operator = new GamepadEx(gamepad2);
        driver = new GamepadEx(gamepad1);

        grippingButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        wristButton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        slowModeButton = new TriggerReader(operator, GamepadKeys.Trigger.RIGHT_TRIGGER);

    }
}
