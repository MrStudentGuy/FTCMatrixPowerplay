package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorHigh extends CommandBase {

    private Elevator elevator;

    public ElevatorHigh(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.extendToHigh();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
