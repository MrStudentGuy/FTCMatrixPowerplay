package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;

public class WristSafe extends CommandBase {

    EndEffector endEffector;

    public WristSafe(EndEffector endEffector){
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.goInit();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
