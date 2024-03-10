package org.firstinspires.ftc.teamcode;
//NOTE TO SELF: STRAFE IN THE X AXIS MOVES HALF THAN YOU'D EXPECT SO J PUT DOUBLE THE DISTANCE :)

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.core.Scalar;


@Autonomous(name = "AABlueClose", group = "Autonomous")
public class AABlueClose extends LinearOpMode {
    private class AWrist{
        private Servo wristGripper;
        public AWrist(HardwareMap hardwareMap){
            wristGripper = hardwareMap.servo.get("wrist_gripper");
        }
        public class ToGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristGripper.setDirection(Servo.Direction.FORWARD);
                wristGripper.setPosition(0.65);
                return false;
            }

        }
        public class ToUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristGripper.setDirection(Servo.Direction.FORWARD);
                wristGripper.setPosition(0.8);
                return false;
            }

        }
        public Action toGround(){
            return new ToGround();
        }
        public Action toUp(){
            return new ToUp();
        }
    }
    public class AGripper {
        private Servo leftGripper;
        private Servo rightGripper;

        public AGripper(HardwareMap hardwareMap){
            leftGripper = hardwareMap.servo.get("leftGripper");
            rightGripper = hardwareMap.servo.get("rightGripper");
        }
        public class OpenLeftGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                leftGripper.setDirection(Servo.Direction.REVERSE);
                leftGripper.setPosition(0.23);

                return false;
            }

        }
        public class CloseLeftGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                leftGripper.setDirection(Servo.Direction.REVERSE);
                leftGripper.setPosition(0.45);
                return false;
            }

        }
        public class OpenRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.48);
                return false;
            }

        }
        public class CloseRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.25);
                return false;
            }

        }
        public Action openLeftGripper() {
            return new OpenLeftGripper();
        }
        public Action closeRightGripper() {
            return new CloseRightGripper();
        }
        public Action openRightGripper() {
            return new OpenRightGripper();
        }
        public Action closeLeftGripper() {
            return new CloseLeftGripper();
        }



    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 72, ( 3*Math.PI)/2);

            VisionBlueClose visionBlueClose = new VisionBlueClose(hardwareMap);
            AGripper gripper = new AGripper(hardwareMap);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            AWrist wrist = new AWrist(hardwareMap);

            waitForStart();


        Action first;
        Action second;
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        int zone = visionBlueClose.elementDetection(telemetry,new Scalar(0, 0, 255, 1));



        Action drop1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(12,72))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(12,27))
                .waitSeconds(0.4)
                .build();
        Action drop1_2 = drive.actionBuilder(new Pose2d(12,27,(3*Math.PI)/2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(15,71.5))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(110,71.5))
                .build();
        Action drop2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-7,72))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-7,14))
                .waitSeconds(0.4)
                .build();
        Action drop2_2 = drive.actionBuilder(new Pose2d(-7,14,(3*Math.PI)/2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-7,71.5))
                .waitSeconds(1)
                .strafeTo(new Vector2d(110,71.5))
                .build();
        Action drop3 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0,20))
                .waitSeconds(0.2)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-15,15))
                .build();
        Action drop3_2 = drive.actionBuilder(new Pose2d(-15,15,(Math.PI)))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-5,20))
                .waitSeconds(0.2)
                .turn(Math.toRadians(90))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-5,71.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(110,71.5))
                .build();

        Action wait = drive.actionBuilder(new Pose2d(110,71.5, (3*Math.PI)/2))
                .waitSeconds(1)
                .build();

        if (zone == 3) {
            first = drop3;
            second = drop3_2;
        } else if (zone == 2) {
            first = drop2;
            second = drop2_2;
        } else {
            first = drop1;
            second = drop1_2;
        }



        Actions.runBlocking(new SequentialAction(
                        gripper.closeLeftGripper(),
                        gripper.closeRightGripper(),
                        wrist.toGround(),
                        first,
                        gripper.openRightGripper(),
                        second,
                        gripper.openLeftGripper(),
                        wait,
                        wrist.toUp(),
                        wait
                )
        );


            //magic vision
            /*Actions.runBlocking(

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(60, -72), 0)
                            .build());


                            */


    }
}
