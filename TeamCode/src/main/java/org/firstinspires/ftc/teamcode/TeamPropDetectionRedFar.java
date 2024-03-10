package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetectionRedFar extends OpenCvPipeline {
    Mat zone1;
    Mat zone2;
    Mat zone3;
    Scalar avgColor1;
    Scalar avgColor2;
    Scalar avgColor3;
    double difference1;
    double difference2;
    Scalar avgColor2NoSpike = new Scalar(213,199,198,1);
    Scalar avgColor3NoSpike = new Scalar(228,187,169,1);
    double difference3;
    Scalar allianceColor = new Scalar(255,0,0,1);
    //betting that it's left zone in case nothing works
    static int zone = 1;
    Mat original;
 double avg2;
 double avg3;
    @Override
    public Mat processFrame(Mat input) {
        original = input.clone();
        //change values when we have the camera to fit the lines
        zone2 = input.submat(new Rect(130, 100, 260, 140));
        zone3 = input.submat(new Rect(500, 150, 140, 260));
        //Averaging the colors in the zones
        avgColor2 = Core.mean(zone2);
        avgColor3 = Core.mean(zone3);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);
        difference2 = colorDifference(avgColor2, allianceColor);
        difference3 = colorDifference(avgColor3, allianceColor);
        if ((difference2< (colorDifference(avgColor2NoSpike,allianceColor)-30)) && (difference2 < difference3)) {
            zone = 2;
        } else if ((difference3< (colorDifference(avgColor3NoSpike, allianceColor)-50)) && (difference3 < difference2)) {
            zone = 3;
        }
        else{
            zone = 1;
        }
        return input;
    }
    public void setAlliance(Scalar alliance){
            allianceColor = alliance;
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