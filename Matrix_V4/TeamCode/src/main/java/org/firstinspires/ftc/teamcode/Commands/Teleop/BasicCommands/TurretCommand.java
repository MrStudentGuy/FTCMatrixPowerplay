package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
public class TurretCommand extends CommandBase {

    Turret turret;
    Elevator elevator;

    double turretDegrees;
    public static double maxTurretVel = 180, maxTurretAccel = 180, maxTurretJerk = 180;

    public TurretCommand(Turret turret, Elevator elevator, double turretDegrees){
        this.turret = turret;
        this.elevator = elevator;
        addRequirements(turret);
        this.turretDegrees = turretDegrees;
    }

    public TurretCommand(Turret turret, Elevator elevator, double turretDegrees, double maxTurretVel, double maxTurretAccel, double maxTurretJerk){
        this.turret = turret;
        this.elevator = elevator;
        addRequirements(turret);
        this.turretDegrees = turretDegrees;
        this.maxTurretAccel = maxTurretAccel;
        this.maxTurretJerk = maxTurretJerk;
        this.maxTurretVel = maxTurretVel;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setDegree(turretDegrees, maxTurretVel, maxTurretAccel, maxTurretJerk);
    }

    @Override
    public boolean isFinished() {
        boolean isFinishedFlag = false;
        if(elevator.getPosition()[0] > elevator.POSITIONS[elevator.LOW_POLE]-10){
            isFinishedFlag = true;
        }
        else{
            isFinishedFlag = false;
        }
        return isFinishedFlag;
    }
}
