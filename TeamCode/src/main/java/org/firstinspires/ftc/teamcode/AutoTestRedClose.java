package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;

@Autonomous(name = "Auto Test Red Close", group = "IterativeOpMode")//iterative maybe?
public class AutoTestRedClose extends LinearOpMode {
        /*
         * EDIT THESE PARAMETERS AS NEEDED
         */
        final boolean USING_WEBCAM = false;
        final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
        final int RESOLUTION_WIDTH = 640;
        final int RESOLUTION_HEIGHT = 480;
        int zone = 0;
        //private Arm arm;

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
            zone = vision.elementDetection(telemetry, new Scalar(255, 0, 0, 1));
            telemetry.addData("Element Zone", zone);
            telemetry.update();
            /*

            Trajectory halftileForward = drive.trajectoryBuilder(new Pose2d())
                    .forward(12)
                    .build();
            Trajectory halftileRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(24)
                    .build();*/
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d beginning = new Pose2d(12, -60, Math.toRadians(90));
            Pose2d postConeRedBackAlliance = new Pose2d(12, -12, 0);
            Trajectory zone1_strafe = drive.trajectoryBuilder(beginning)
                    .strafeLeft(3)
                    .build();
            Trajectory zone1_forward = drive.trajectoryBuilder(zone1_strafe.end())
                    .back(24)
                    .build();
            Trajectory zone2_strafe = drive.trajectoryBuilder(beginning)
                    .strafeLeft(5)
                    .build();
            Trajectory zone2_forward = drive.trajectoryBuilder(zone2_strafe.end())
                    .back(28)
                    .build();
            Trajectory zone3_forward = drive.trajectoryBuilder(beginning)
                    .back(20)
                    .build();
            Trajectory zone1ToBackboard = drive.trajectoryBuilder(beginning)
                    .forward(36)
                    .build();
            Trajectory zone2ToBackboard = drive.trajectoryBuilder(beginning)
                    .forward(30)
                    .build();
            Trajectory zone3ToBackboard = drive.trajectoryBuilder(beginning)
                    .forward(26)
                    .build();

//            Trajectory commonToBackboard_spline = drive.trajectoryBuilder(postConeRedBackAlliance)
//                    .splineTo(new Vector2d((12), -12), Math.toRadians(-90))
//                    .build();
            Trajectory commonToBackboard_forward = drive.trajectoryBuilder(postConeRedBackAlliance)
                    .back(24)
                    .build();
            Trajectory commonToBackboard_strafe = drive.trajectoryBuilder(commonToBackboard_forward.end())
                    .strafeLeft(24)
                    .build();
            Trajectory commonToBackboard_forward2 = drive.trajectoryBuilder(commonToBackboard_strafe.end())
                    .back(12)
                    .build();

            Trajectory park_strafe = drive.trajectoryBuilder(postConeRedBackAlliance)
                    .strafeRight(30)
                    .build();
//            Trajectory park_spline = drive.trajectoryBuilder(park_strafe.end())
//                    .splineTo(new Vector2d(48, -36), Math.toRadians(-90))
//                    .build();
            Trajectory park_back = drive.trajectoryBuilder(park_strafe.end())
                    .forward(12)
                    .build();

            //PUT PURPLE PIXEL IN RIGHT GRIPPER
            waitForStart();
            while(opModeIsActive()) {
                drive.setPoseEstimate(beginning);
                //teamPropDetection.processFrame();
                //zone = teamPropDetection.getZone();

                if(zone==1){
                    drive.followTrajectory(zone1_strafe);
                    drive.followTrajectory(zone1_forward);
                    drive.turn(Math.toRadians(-93));
                    //arm.openRight();
                    drive.followTrajectory(zone1ToBackboard);
                    //arm.MagicliftArm
                    //arm.openLeft();
                    //arm.MagiclowerArm
                    drive.followTrajectory(park_strafe);
//                    drive.followTrajectory(park_spline);
                    drive.followTrajectory(park_back);
                } else if (zone==2) {
                    drive.followTrajectory(zone2_strafe);
                    drive.followTrajectory(zone2_forward);
                    //arm.openRight();
                    drive.turn(Math.toRadians(-93));
                    drive.followTrajectory(zone2ToBackboard);
                    //arm.MagicliftArm
                    //arm.openLeft();
                    //arm.MagiclowerArm
                    drive.followTrajectory(park_strafe);
                    drive.followTrajectory(park_back);

                }
                else {
                    drive.followTrajectory(zone3_forward);
                    drive.turn(Math.toRadians(93));
                    drive.turn(Math.toRadians(183));
                    //arm.openRight();
                    drive.followTrajectory(zone3ToBackboard);
                    //arm.MagicliftArm
                    //arm.openLeft();
                    //arm.MagiclowerArm
                    drive.followTrajectory(park_strafe);
                    drive.followTrajectory(park_back);


                }
            }


        }

}



