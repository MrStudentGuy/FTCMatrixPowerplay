package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;

import java.util.Objects;

public class ElevatorGripping extends CommandBase {

    Elevator elevator;
    Turret turret;
//    EndEffector endEffector;

    public ElevatorGripping(Elevator elevator, Turret turret){
        this.elevator = elevator;
        this.turret = turret;
//        this.endEffector = endEffector;
        addRequirements(elevator);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        super.initialize();

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        elevator.extendToGripping();
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(turret.getDegree()) > 10 && !Objects.equals(EndEffector.wristState, "INIT")){
            turret.setDegree(0);
            return false;
        }
        else{
            return true;
        }
    }
}
