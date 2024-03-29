package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionMagic extends OpenCvPipeline {
    static int zone = 1;
    static double percentDifference;
    Mat original;
    static double satZone2;
    static double satZone3;
    public double MinDif = 45;
    public double MinZone2 = 35;
    public double MinZone3 = 60;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        try {
            Thread.sleep( 500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        original = input.clone();
        Rect zone2 = new Rect(70, 100, 320, 140);
        Rect zone3 = new Rect(450, 150, 190, 260);
        satZone2 = getAvgSaturation(hsvMat, zone2);
        satZone3 = getAvgSaturation(hsvMat, zone3);
        original.submat(zone2).setTo(new Scalar(255,0,0,0.5));
        original.submat(zone3).setTo(new Scalar(255,0,0,0.5));
        percentDifference = getPercentDifference(satZone2,satZone3);
        if ( satZone3 > satZone2 && satZone3 > MinZone3) {
            zone = 3;
        } else if (satZone2 > satZone3 && satZone2 > MinZone2) {
            zone = 2;
        } else if (percentDifference <= MinDif) {
            zone = 1;
        }
        return original;
    }
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }
    private double getPercentDifference(double val1, double val2) {
        return Math.abs(val1 - val2) / ((val1 + val2) / 2) * 100;
    }
    public int getZone(){
        return zone;
    }
    public double getSatZone2(){
        return satZone2;
    }
    public double getSatZone3(){
        return satZone3;
    }
    public double getPercentDifference(){
        return percentDifference;
    }
}