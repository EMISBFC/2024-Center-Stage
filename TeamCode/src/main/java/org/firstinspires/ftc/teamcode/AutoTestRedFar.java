package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Auto Test", group = "IterativeOpMode")//iterative maybe?
public class AutoTestRedFar extends LinearOpMode {
        /*
         * EDIT THESE PARAMETERS AS NEEDED
         */
        final boolean USING_WEBCAM = false;
        final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
        final int RESOLUTION_WIDTH = 640;
        final int RESOLUTION_HEIGHT = 480;
        int zone = 0;
        private Arm arm;

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
            /*

            Trajectory halftileForward = drive.trajectoryBuilder(new Pose2d())
                    .forward(12)
                    .build();
            Trajectory halftileRight = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(24)
                    .build();*/
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Pose2d beggining = new Pose2d(-36, -60, Math.toRadians(90));
            Pose2d postConeRedBackAlliance = new Pose2d(-36, -12, 0);
            Trajectory zone1 = drive.trajectoryBuilder(beggining)
                    .forward(20)
                    .splineTo(new Vector2d((-36), -40), Math.toRadians(90))
                    .build();
            Trajectory zone2 = drive.trajectoryBuilder(beggining)
                    .forward(22)
                    .build();
            Trajectory zone3 = drive.trajectoryBuilder(beggining)
                    .forward(24)
                    .splineTo(new Vector2d((-36), -36), Math.toRadians(-90))
                    .build();
            Trajectory zone1ToCommon = drive.trajectoryBuilder(beggining)
                    .back(2)
                    .splineTo(new Vector2d((-34), -40), Math.toRadians(-90))
                    .strafeLeft(2)
                    .forward(28)
                    .build();
            Trajectory zone2ToCommon = drive.trajectoryBuilder(beggining)
                    .forward(26)
                    .build();
            Trajectory zone3ToCommon = drive.trajectoryBuilder(beggining)
                    .back(2)
                    .splineTo(new Vector2d((-36), -40), Math.toRadians(90))
                    .strafeRight(2)
                    .forward(24)
                    .build();

            Trajectory commonToBackboard = drive.trajectoryBuilder(postConeRedBackAlliance)
                    .splineTo(new Vector2d((-36), -12), Math.toRadians(-90))
                    .forward(72)
                    .strafeRight(24)
                    .forward(12)
                    .build();
            Trajectory park = drive.trajectoryBuilder(postConeRedBackAlliance)
                    .strafeRight(24)
                    .splineTo(new Vector2d(48, -36), Math.toRadians(-90))
                    .back(24)
                    .build();

            //PUT PURPLE PIXEL IN RIGHT GRIPPER
            while(opModeIsActive()) {
                //teamPropDetection.processFrame();
                //zone = teamPropDetection.getZone();

                if(zone==1){
                    drive.followTrajectory(zone1);
                    arm.openRight();
                    drive.followTrajectory(zone1ToCommon);
                    drive.followTrajectory(commonToBackboard);
                    //arm.MagicliftArm
                    arm.openLeft();
                    //arm.MagiclowerArm
                    drive.followTrajectory(park);
                } else if (zone==2) {
                    drive.followTrajectory(zone2);
                    arm.openRight();
                    drive.followTrajectory(zone2ToCommon);
                    drive.followTrajectory(commonToBackboard);
                    //arm.MagicliftArm
                    arm.openLeft();
                    //arm.MagiclowerArm
                    drive.followTrajectory(park);

                }
                else {
                    drive.followTrajectory(zone3);
                    arm.openRight();
                    drive.followTrajectory(zone3ToCommon);
                    drive.followTrajectory(commonToBackboard);
                    //arm.MagicliftArm
                    arm.openLeft();
                    //arm.MagiclowerArm
                    drive.followTrajectory(park);


                }
            }

            //public void moveToBackboardFromBack()
        }

}



