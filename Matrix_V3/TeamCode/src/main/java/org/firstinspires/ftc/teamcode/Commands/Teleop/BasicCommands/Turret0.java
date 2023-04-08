package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class Turret0 extends CommandBase {

    Turret turret;


    public Turret0(Turret turret){
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        super.initialize();
        turret.setDegree(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
