package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;

public class WristUp extends CommandBase {

    EndEffector endEffector;

    public WristUp(EndEffector endEffector){
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.goTop();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
