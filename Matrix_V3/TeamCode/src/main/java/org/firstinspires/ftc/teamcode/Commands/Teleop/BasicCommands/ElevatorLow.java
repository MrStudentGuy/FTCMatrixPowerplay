package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class ElevatorLow extends CommandBase {

    Elevator elevator;


    public ElevatorLow(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        elevator.extendToLow();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
