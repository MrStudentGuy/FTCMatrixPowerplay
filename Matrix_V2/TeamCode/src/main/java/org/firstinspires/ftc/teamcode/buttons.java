package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

public class buttons {
    GamepadEx gamepad;
    public buttons(GamepadEx gamepadEx){
        gamepad = gamepadEx;
        A = new ButtonReader(gamepadEx, GamepadKeys.Button.A);
        B = new ButtonReader(gamepadEx, GamepadKeys.Button.B);
        X = new ButtonReader(gamepadEx, GamepadKeys.Button.X);
        Y = new ButtonReader(gamepadEx, GamepadKeys.Button.Y);
        DOWN = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_DOWN);
        UP = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_UP);
        LEFT = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_LEFT);
        RIGHT = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_RIGHT);
        R1 = new ButtonReader(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);
        R3 = new ButtonReader(gamepadEx, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        L1 = new ButtonReader(gamepadEx, GamepadKeys.Button.LEFT_BUMPER);
        L3 = new ButtonReader(gamepadEx, GamepadKeys.Button.LEFT_STICK_BUTTON);
        BACK = new ButtonReader(gamepadEx, GamepadKeys.Button.BACK);
        START = new ButtonReader(gamepadEx, GamepadKeys.Button.START);
        R2 = new TriggerReader(gamepadEx, GamepadKeys.Trigger.LEFT_TRIGGER);
        L2 = new TriggerReader(gamepadEx, GamepadKeys.Trigger.RIGHT_TRIGGER);

    }

    public ButtonReader A, B, X, Y, DOWN, UP, LEFT, RIGHT, R1, R3, L1, L3, BACK, START;
    public TriggerReader R2, L2;
}
