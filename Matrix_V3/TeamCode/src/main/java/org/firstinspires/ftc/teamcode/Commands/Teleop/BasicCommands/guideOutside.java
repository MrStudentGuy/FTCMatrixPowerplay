package org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.Subsystems.Guide;

public class guideOutside extends CommandBase {
    Guide guide;
    EndEffector endEffector;

    public guideOutside(Guide guide, EndEffector endEffector){
        this.endEffector = endEffector;
        this.guide = guide;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        guide.outside();
    }

    @Override
    public boolean isFinished() {
        return endEffector.gripperStateSupplier.getAsInt() == 0;
    }
}
