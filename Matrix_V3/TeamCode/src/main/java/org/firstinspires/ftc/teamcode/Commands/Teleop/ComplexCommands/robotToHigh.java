package org.firstinspires.ftc.teamcode.Commands.Teleop.ComplexCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.ElevatorHigh;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.WristUp;
import org.firstinspires.ftc.teamcode.Commands.Teleop.BasicCommands.guideOutside;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.Subsystems.Guide;

public class robotToHigh extends SequentialCommandGroup {

    public robotToHigh(Elevator elevator, EndEffector endEffector, Guide guide){
        addCommands(new ElevatorHigh(elevator), new WristUp(endEffector), new guideOutside(guide, endEffector));
        addRequirements(elevator, endEffector, guide);
    }
}
