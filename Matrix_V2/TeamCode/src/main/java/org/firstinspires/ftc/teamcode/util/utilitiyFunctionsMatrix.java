package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;

public class utilitiyFunctionsMatrix {

    public static Pose2d previousPose = new Pose2d(0,0,0);

    private static boolean firstFlag = false;
    private static ElapsedTime velocityEstimatorTimer;

    public static double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2*Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }

        return radians;
    }


    public static Pose2d velocityEstimator(Pose2d currentPose){
        if(!firstFlag){
            previousPose = currentPose;
            velocityEstimatorTimer = new ElapsedTime();
            firstFlag = true;
        }
        double deltaD_x = currentPose.getX() - previousPose.getX();
        double deltaD_y = currentPose.getY() - previousPose.getY();
        double deltaD_theta = currentPose.getHeading() - previousPose.getHeading();
        double delta_T = velocityEstimatorTimer.milliseconds();
        Pose2d EstimatedVelocity = new Pose2d(deltaD_x/delta_T, deltaD_y/delta_T, deltaD_theta/delta_T);


        previousPose = currentPose;
        velocityEstimatorTimer.reset();

        return EstimatedVelocity;
    }


    public class LowPassFilter{
        private double previousValue = 0;
        private double Alpha = 0;
        LowPassFilter(double alpha){
            Alpha = alpha;
        }

        public double filter(double currentValue){
            double filtered_value = (Alpha * currentValue) + ((1-Alpha)*previousValue);
            previousValue = filtered_value;
            return filtered_value;
        }

        public void setAlpha(double alpha){
            Alpha = alpha;
        }
    }
}
