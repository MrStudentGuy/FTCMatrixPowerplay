package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorMid extends CommandBase {

    Elevator elevator;

    public ElevatorMid(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.extendToMid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
