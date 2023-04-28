package org.firstinspires.ftc.teamcode.Vision.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class testPipeline extends OpenCvPipeline {

    Mat hsv = new Mat();
    Mat mask  = new Mat();
    Mat edges = new Mat();

    @Override
    public Mat processFrame(Mat input){

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Scalar lowHSV = new Scalar(20, 50, 255);
        Scalar highHSV = new Scalar(40, 255, 255);

        Core.inRange(input, lowHSV, highHSV, mask);



//        Imgproc.Canny(mask, edges, 100, 300);

        return mask;
    }
}
