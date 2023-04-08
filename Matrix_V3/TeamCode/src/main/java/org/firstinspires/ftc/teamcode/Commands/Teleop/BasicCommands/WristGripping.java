package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;

public class WristGripping extends CommandBase {

    private EndEffector endEffector;

    public WristGripping(EndEffector endEffector){
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        super.initialize();
        endEffector.goGripping();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
