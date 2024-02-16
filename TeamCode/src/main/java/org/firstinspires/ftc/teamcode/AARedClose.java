package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;


@Autonomous(name = "AARedClose", group = "Autonomous")
public class AARedClose extends LinearOpMode {

    public class AGripper {
        private Servo leftGripper;
        private Servo rightGripper;

        public AGripper(HardwareMap hardwareMap){
            leftGripper = hardwareMap.servo.get("leftGripper");
            rightGripper = hardwareMap.servo.get("rightGripper");
        }
        public class OpenGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftGripper.setDirection(Servo.Direction.FORWARD);
                leftGripper.setPosition(0.3);
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.75);
                return false;
            }

        }
        public Action openGripper() {
            return new OpenGripper();
        }



    }
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(7, -72, Math.PI/2);
        AGripper gripper = new AGripper(hardwareMap);


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();


        /*Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(7, -36), (Math.PI)/2).build(),
                drive.actionBuilder(new Pose2d(7, -36, (Math.PI)/2))
                        .splineTo(new Vector2d(7, -72), (Math.PI)/2).build(),
                drive.actionBuilder(new Pose2d(7, -72, (Math.PI)/2))
                        .splineTo(new Vector2d(60, -72), (Math.PI)/2).build()
                //gripper.openGripper()
                    )
            );*/
        Action  move;
        Action  back;
        Action  park;
        move = drive.actionBuilder(beginPose)
                .lineToY(-36)
                .waitSeconds(2)
                .lineToY(-72)
                .strafeTo(new Vector2d(60, -72))
                .build();

        Actions.runBlocking(new SequentialAction(
                        move
                )

                        //gripper.openGripper()

        );


    }
}
