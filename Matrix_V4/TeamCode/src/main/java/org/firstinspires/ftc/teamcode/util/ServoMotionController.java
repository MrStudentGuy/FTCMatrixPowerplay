package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoMotionController {
    private Servo servo;
    private double targetPosition;
    private double currentPosition;
    private double movementSpeed;

    public ServoMotionController(Servo servo, double movementSpeed) {
        this.servo = servo;
        this.movementSpeed = movementSpeed;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setMovementSpeed(double speed){
        this.movementSpeed = speed;
    }

    public void update() {
        if (currentPosition < targetPosition) {
            currentPosition += movementSpeed;
            if (currentPosition > targetPosition) {
                currentPosition = targetPosition;
            }
        } else if (currentPosition > targetPosition) {
            currentPosition -= movementSpeed;
            if (currentPosition < targetPosition) {
                currentPosition = targetPosition;
            }
        }

        servo.setPosition(currentPosition);
    }
}
