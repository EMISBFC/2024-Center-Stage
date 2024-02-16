
/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AABlueFar", group = "LinearOpMode")
public class AABlueFar extends LinearOpMode {
    private Servo rightGripper;
    private Servo leftGripper;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        rightGripper = hardwareMap.servo.get("rightGripper");
        leftGripper = hardwareMap.servo.get("leftGripper");
        Trajectory right = drive.trajectoryBuilder(new Pose2d(-60, -30, Math.toRadians(0)))
                .strafeRight(67.5)
                .build();

        Trajectory forward = drive.trajectoryBuilder(right.end())
                .forward(87)
                .build();
        Trajectory left = drive.trajectoryBuilder(forward.end())
                .strafeRight(63)
                .build();
        Trajectory park = drive.trajectoryBuilder(left.end())
                .forward(35)
                .build();
        waitForStart();
        while (opModeIsActive()) {
            drive.setPoseEstimate(new Pose2d(-60, -30, Math.toRadians(0)));
            drive.followTrajectory(right);
            drive.followTrajectory(forward);
            drive.followTrajectory(left);
            drive.followTrajectory(park);
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.05);
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.05);
            //open both grippers
        }
    }
}*/