package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class ElevatorStackCone5 extends CommandBase {

    Elevator elevator;

    public ElevatorStackCone5(Elevator elevator){
        this.elevator = elevator;
    }
    @Override
    public void initialize() {
        super.initialize();
        elevator.extendTo(elevator.AUTO_POSITION[4]);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
