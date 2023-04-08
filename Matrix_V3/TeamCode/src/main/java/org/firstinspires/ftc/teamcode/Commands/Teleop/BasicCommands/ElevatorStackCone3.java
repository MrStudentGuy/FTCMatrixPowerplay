package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class ElevatorStackCone3 extends CommandBase {

    Elevator elevator;

    public ElevatorStackCone3(Elevator elevator){
        this.elevator = elevator;
    }
    @Override
    public void initialize() {
        super.initialize();
        elevator.extendTo(elevator.AUTO_POSITION[2]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
