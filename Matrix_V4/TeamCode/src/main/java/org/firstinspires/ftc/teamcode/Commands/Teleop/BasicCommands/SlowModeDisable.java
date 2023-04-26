package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalVars;

public class SlowModeDisable extends CommandBase {

    public SlowModeDisable(){

    }

    @Override
    public void initialize() {
        super.initialize();
        GlobalVars.slowDriveFlag = false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
