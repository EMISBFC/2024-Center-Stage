

package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.ArrayList;


public class AprilTags
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    /*
    Logitech HD Webcam C270, Calibrated by Noah Andrews, 2019.03.13 using 3DF Zephyr
    <Camera vid="Logitech" pid="0x0825">
     <Calibration
    size="640 480"
    focalLength="822.317f, 822.317f"
    principalPoint="319.495f, 242.502f"
    distortionCoefficients="-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0"
     */

    //NEED TO CALIBRATE THE FOLLOWING
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

//id of each tag w their meaning
    int BlueLeft = 1;
    int BlueCenter = 2;
    int BlueRight = 3;
    int RedLeft = 4;
    int RedCenter = 5;
    int RedRight = 6;

    AprilTagDetection tagOfInterest = null;


    public AprilTags(HardwareMap hardwareMap) {

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
    }
    public int AprilTagDetection() { //should return the id of the tag, MIGHT WANNA MAKE IT SO IF WE'RE
        // BLUE WE ONL TRY TO FIND BLUE APRIL TAGS AND VICEVERSA, DEPENDS ON HOW ACCURATE THE READING OF TAGS IS
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        boolean found = false;
        int id = 0;
        while(!found) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == BlueCenter || tag.id == BlueLeft || tag.id == BlueRight || tag.id == RedCenter || tag.id == RedLeft || tag.id == RedRight) {
                    tagOfInterest = tag;
                    found = true;
                    id=tag.id;
                    break;

                }

                 else {
                    telemetry.addLine("Don't see any tag of interest :(");

                }

            }
            sleep(20);
        }
        return id;
    }


    /*void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }*/
}