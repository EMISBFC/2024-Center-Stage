package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;


//all of this code is written assuming we get red alliance
public class TeamPropDetection extends OpenCvPipeline {
    Mat zone1;
    Mat zone2;
    Mat zone3;
    Scalar avgColor1;
    Scalar avgColor2;
    Scalar avgColor3;
    double difference1;
    double difference2;
    double difference3;
    Scalar allianceColor =new Scalar(255, 0, 0, 1);//assume RED
    final int maxDifference = 190; //arbitrary number for now

    //betting that it's left zone in case nothing works
    static int zone = 1;
    Mat original;

    @Override
    public Mat processFrame(Mat input) {
        original = input.clone();

        //change values when we have the camera to fit the lines
        zone1 = input.submat(new Rect(0, 100, 64, 380));
        zone2 = input.submat(new Rect(200, 100, 200, 270));
        zone3 = input.submat(new Rect(500, 100, 140, 380));


        //Averaging the colors in the zones
        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);
        avgColor3 = Core.mean(zone3);

        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);

        difference1 = colorDifference(avgColor1, allianceColor);
        difference2 = colorDifference(avgColor2, allianceColor);
        difference3 = colorDifference(avgColor3, allianceColor);
        if ((difference1 < difference2) && (difference2 < difference3)) {
            zone = 1;
        } else if ((difference2 < difference3) && (difference2 < difference1)) {
            zone = 2;
        }
        else{
            zone = 3;
        }

        /*if ((difference1 > maxDifference) && (difference2 > maxDifference)) {
            zone = 3;
        } else {

            if (difference1 < difference2) {
                zone = 1;
            } else {
                zone = 2;
            }

        }*/
        return input;
    }




    public double colorDifference(Scalar zoneColor, Scalar aimColor){
        double r1 = zoneColor.val[0];
        double g1 = zoneColor.val[1];
        double b1 = zoneColor.val[2];

        double r2 = aimColor.val[0];
        double g2 = aimColor.val[1];
        double b2 = aimColor.val[2];

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }
    //return 1 if left, 2 if center, 3 if right
    public int getZone(){
        return zone;
    }


}
