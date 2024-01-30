package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;

import java.util.Locale;
import java.util.Locale;
@Autonomous(name = "Auto Test", group = "IterativeOpMode")//iterative maybe?
public class AutoTest extends LinearOpMode {
        /*
         * EDIT THESE PARAMETERS AS NEEDED
         */
        final boolean USING_WEBCAM = false;
        final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
        final int RESOLUTION_WIDTH = 640;
        final int RESOLUTION_HEIGHT = 480;
        int zone = 0;

    private Vision vision=null;

        // Internal state
        boolean lastX;
        int frameCount;
        long capReqTime;
        @Override
        public void runOpMode()
        {
            /*VisionPortal portal;
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                    .build();*/
            vision = new Vision(hardwareMap);
            zone = vision.elementDetection(telemetry);
            telemetry.addData("Element Zone", zone);
            telemetry.update();
            /*SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Trajectory halftileForward = drive.trajectoryBuilder(new Pose2d())
                    .forward(12)
                    .build();
            Trajectory halftileRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(24)
                    .build();*/
            waitForStart();
            while(opModeIsActive()) {
                //teamPropDetection.processFrame();
                //zone = teamPropDetection.getZone();

                if(zone==1){

                } else if (zone==2) {

                }
                else {

                }
            }

            //public void moveToBackboardFromBack()
        }

}



