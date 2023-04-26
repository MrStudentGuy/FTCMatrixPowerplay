package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalVars;

public class SlowModeEnable extends CommandBase {

    public SlowModeEnable(){

    }

    @Override
    public void initialize() {
        super.initialize();
        GlobalVars.slowDriveFlag = true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
