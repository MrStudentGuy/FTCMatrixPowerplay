package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class ElevatorStackCone4 extends CommandBase {

    Elevator elevator;

    public ElevatorStackCone4(Elevator elevator){
        this.elevator = elevator;
    }
    @Override
    public void initialize() {
        super.initialize();
        elevator.extendTo(elevator.AUTO_POSITION[3]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
