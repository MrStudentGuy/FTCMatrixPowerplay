package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class Matrix{
    Telemetry robotTelemetry;
    HardwareMap robotHardware;
    public Lift robotLift;
    public Servos robotServos;
    public Sensors robotSensors;
    public Turret robotTurret;

    public Matrix(HardwareMap hardwareMap, Telemetry telemetry){
        robotHardware = hardwareMap;
        robotTelemetry = telemetry;
        
        robotLift = new Lift(robotHardware, robotTelemetry);
        robotServos = new Servos(robotHardware, robotTelemetry);
        robotTurret = new Turret(robotHardware, "turret", robotTelemetry);
        robotSensors = new Sensors(robotHardware, robotTelemetry);

    }
}
