package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.CloseGripper;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorGripping;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorHigh;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorLow;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorMid;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.OpenGripper;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.Turret0;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.Turret180;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.WristGripping;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.WristSafe;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.WristUp;
import org.firstinspires.ftc.teamcode.GlobalVars;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import java.util.function.BooleanSupplier;

@TeleOp
public class Teleop extends CommandOpMode {

    Gamepad.RumbleEffect resetHeadingEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.00, 0.00, 100)
            .addStep(0.0, 0.0, 50)
            .addStep(0.00, 1.00, 50)
            .addStep(0.0, 0.0, 50)
            .addStep(1.00, 0.00, 100)
            .addStep(0.0, 0.0, 50)
            .addStep(0.00, 1.00, 50)
            .addStep(0.0, 0.0, 50)
            .build();
    ElapsedTime timer;
    BooleanSupplier traditionalFlagSupplier = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return GlobalVars.traditionalFlag;
        }
    };


    //Subsystems
    Elevator elevator;
    EndEffector endEffector;
    Turret turret;
    Slider slider;
    Drive drive;

    //Commands




    //Gamepad stuff
    GamepadEx driver, operator;
    Button safeButton, traditionalButton, liftHighButton, liftMidButton, liftLowButton, liftGrippingButton, grippingButton, turret180Button, turret0Button, wristButton, turretToggleButton;

    @Override
    public void initialize() {
        PhotonCore.enable();
        GlobalVars.currentOpModeType = GlobalVars.runMode.TELEOP;
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        //Subsystem Declaration
        this.endEffector = new EndEffector(hardwareMap, telemetry);
        this.turret = new Turret(hardwareMap, "turret", telemetry);
        this.elevator = new Elevator(hardwareMap, telemetry, turret.turretDegreesSupplier);
        this.slider = new Slider(hardwareMap, telemetry, this);
        this.drive = new Drive(hardwareMap, telemetry, this);

        //Command Declaration


        //Joystick & Button Declarations
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        liftHighButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        liftMidButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);
        liftLowButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        liftGrippingButton = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);
        grippingButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        wristButton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        safeButton = new GamepadButton(driver, GamepadKeys.Button.A);
//        turret180Button = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
//        turret0Button = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
        turretToggleButton = new GamepadButton(driver, GamepadKeys.Button.LEFT_STICK_BUTTON);
        traditionalButton = new GamepadButton(driver, GamepadKeys.Button.BACK);


        //Command Scheduling
        schedule(new SequentialCommandGroup(new CloseGripper(endEffector),new WristGripping(endEffector), new WaitCommand(1500), new ElevatorGripping(elevator, turret)));
        liftHighButton.whenPressed(new ParallelCommandGroup(new ElevatorHigh(elevator), new WristUp(endEffector)));
        liftMidButton.whenPressed(new ParallelCommandGroup(new ElevatorMid(elevator), new WristUp(endEffector)));
        liftLowButton.whenPressed(new ParallelCommandGroup(new ElevatorLow(elevator), new WristUp(endEffector)));
        liftGrippingButton.whenPressed(new ParallelCommandGroup(new ElevatorGripping(elevator, turret), new WristGripping(endEffector)));
        safeButton.whenPressed(new SequentialCommandGroup(new CloseGripper(endEffector), new WaitCommand(50), new WristSafe(endEffector), new WaitCommand(100), new InstantCommand(()->elevator.extendTo(0))));
//        turret180Button.whenPressed(new Turret180(turret));
//        turret0Button.whenPressed(new Turret0(turret));
        turretToggleButton.toggleWhenPressed(new Turret180(turret), new Turret0(turret));
        grippingButton.toggleWhenPressed(
                new ConditionalCommand(new OpenGripper(endEffector), new SequentialCommandGroup(new OpenGripper(endEffector), new WaitCommand(100), new ParallelCommandGroup(new ElevatorGripping(elevator, turret), new WristGripping(endEffector))), traditionalFlagSupplier),
                new ConditionalCommand(new CloseGripper(endEffector), new SequentialCommandGroup(new CloseGripper(endEffector),new WaitCommand(250), new ElevatorLow(elevator)), traditionalFlagSupplier)
        );
        wristButton.toggleWhenPressed(new WristGripping(endEffector), new WristUp(endEffector));
        traditionalButton.toggleWhenPressed(new InstantCommand(()->GlobalVars.traditionalFlag = true), new InstantCommand(()->GlobalVars.traditionalFlag = false));
        register(drive);
        register(slider);
    }


    @Override
    public void run() {
        super.run();
        if(timer == null){
            drive.startIMUThread();
            timer = new ElapsedTime();
        }

        boolean touchpadPressed = gamepad1.touchpad;
        if(touchpadPressed){
            drive.resetHeading();
            gamepad1.runRumbleEffect(resetHeadingEffect);
        }

//        if(timer.seconds() > )


    }
}
