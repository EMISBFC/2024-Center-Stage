package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AABlueClose", group = "LinearOpMode")
public class AABlueClose extends LinearOpMode {
    GripperTest gripperTest;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        gripperTest = new GripperTest(hardwareMap);
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(-60, 15, Math.toRadians(0)))
                .forward(75)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d(-60, 15, Math.toRadians(0)))
                .strafeRight(75)
                .build();

        waitForStart();
        while (opModeIsActive()) {
            drive.setPoseEstimate(new Pose2d(-60, 15, Math.toRadians(0)));

            drive.followTrajectory(left);
            gripperTest.openLeft();
            gripperTest.openRight();
            break;

            //open both grippers
        }
    }
}