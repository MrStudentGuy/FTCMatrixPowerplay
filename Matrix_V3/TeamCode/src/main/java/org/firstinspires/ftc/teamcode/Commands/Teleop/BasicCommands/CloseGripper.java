package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;

public class CloseGripper extends CommandBase {

    private EndEffector endEffector;


    public CloseGripper(EndEffector endEffector){
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        super.initialize();
        endEffector.closeGripper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
