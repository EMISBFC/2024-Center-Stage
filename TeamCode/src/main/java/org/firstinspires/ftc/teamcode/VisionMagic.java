 //For future years the logic can be kept simply tweak the positions of the "boxes" and the MinZone of each
package org.firstinspires.ftc.teamcode;
//The MinZone3 is made a static in constants as it is changed for BlueClose 
import static org.firstinspires.ftc.teamcode.Constants.MinZone3;

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

    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        try {
            Thread.sleep( 500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        //switch the processing in OpenCV from RGB to saturation so you can use Core.mean
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        //make a copy of the input to draw in
        original = input.clone();
        //define the two zones
        Rect zone2 = new Rect(70, 100, 320, 140);
        Rect zone3 = new Rect(450, 150, 190, 260);
        //run the core.mean thing
        satZone2 = getAvgSaturation(hsvMat, zone2);
        satZone3 = getAvgSaturation(hsvMat, zone3);
        //draw in the original (copy) matrix two red boxes where the zones are so they can be seen in camerastream
        original.submat(zone2).setTo(new Scalar(255,0,0,1));
        original.submat(zone3).setTo(new Scalar(255,0,0,1));
    
        percentDifference = getPercentDifference(satZone2,satZone3);

        //the comparison to minzone3 is necessary to make sure that there is an object
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
