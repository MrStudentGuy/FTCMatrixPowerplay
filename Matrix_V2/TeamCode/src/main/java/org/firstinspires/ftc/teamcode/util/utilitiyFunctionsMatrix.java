package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;

public class utilitiyFunctionsMatrix {

    /**
     * A class that limits the rate of change of an input value. Useful for implementing voltage, setpoint or output ramps.
     */
    public class SlewRateLimiter{
        private ElapsedTime timer = new ElapsedTime();
        private final double m_positiveRateLimit;
        private final double m_negativeRateLimit;
        private double m_prevVal;
        private double m_prevTime;

        /**
         * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial value
         * @param positiveRateLimit The rate of change limit in the positive direction, in units per millisecond. This is expected to be positive.
         * @param negativeRateLimit The rate of change limit in the negative direction, in units per millisecond. This is expected to be negative.
         * @param initialValue The initial value of the input
         */
        public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue){
            timer.reset();
            m_positiveRateLimit = positiveRateLimit;
            m_negativeRateLimit = negativeRateLimit;
            m_prevVal = initialValue;
            m_prevTime = timer.milliseconds();
        }

        /**
         * Creates a new SlewRateLimiter with the given rateLimit
         * @param rateLimit
         */
        public SlewRateLimiter(double rateLimit) {
               this(rateLimit, -rateLimit, 0);
        }

        /**
         * Filters the input to limit it's slew rate
         * @param input The input value who's slew rate is to be limited
         * @return The filtered Value, which will not change faster than the slew rate.
         */
        public double calculate(double input){
            double currentTime = timer.milliseconds();
            double elapsedTime = currentTime - m_prevTime;

            m_prevVal += Range.clip(
                    input - m_prevVal,
                    m_negativeRateLimit * elapsedTime,
                    m_positiveRateLimit * elapsedTime);
            m_prevTime = currentTime;
            return m_prevVal;
        }

        /**
         * Resets the slew rate limiter to the specified value, while ignoring the rate limit.
         * @param value The value to reset to
         */

        public void reset(double value){
            m_prevVal = value;
            m_prevTime = timer.milliseconds();
        }
    }

    public static Pose2d previousPose = new Pose2d(0,0,0);

    private static boolean firstFlag = false;
    private static ElapsedTime velocityEstimatorTimer;


    /**
     * Wrap the angle to beyond +- 180 degrees/ +- Pi Radians
     * @param radians The current angle measured
     * @return The angle extended according to requirements
     */
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
