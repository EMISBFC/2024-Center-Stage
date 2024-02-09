
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;

@Autonomous(name = "AARedClose", group = "LinearOpMode")
public class AARedClose extends LinearOpMode {
    private Servo rightGripper;
    private Servo leftGripper;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        rightGripper = hardwareMap.servo.get("rightGripper");
        leftGripper = hardwareMap.servo.get("leftGripper");
        Trajectory forward = drive.trajectoryBuilder(new Pose2d(60, 15, Math.toRadians(0)))
                .forward(75)
                .build();


        waitForStart();
        //while (opModeIsActive()) {
            drive.setPoseEstimate(new Pose2d(60, 15, Math.toRadians(0)));

            drive.followTrajectory(forward);
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.95);
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.95);

            //open both grippers
        //}
    }
}