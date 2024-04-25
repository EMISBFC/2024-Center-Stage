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

@Autonomous(name = "AARedClose", group = "Autonomous")
public class AARedClose extends LinearOpMode {
    private class AWrist{
        private Servo wristGripper;
        public AWrist(HardwareMap hardwareMap){
            wristGripper = hardwareMap.servo.get("wrist_gripper");
        }
        public class ToGround implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristGripper.setDirection(Servo.Direction.FORWARD);
                wristGripper.setPosition(Constants.wristFloorPos);
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
                leftGripper.setPosition(Constants.leftGripperClose);
                return false;
            }
        }
        public class CloseLeftGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftGripper.setDirection(Servo.Direction.REVERSE);
                leftGripper.setPosition(Constants.leftGripperOpen);
                return false;
            }
        }
        public class OpenRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(Constants.rightGripperClose);
                return false;
            }
        }
        public class CloseRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(Constants.rightGripperOpen);
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
        Pose2d beginPose = new Pose2d(0, -72, (Math.PI)/2);

        VisionRedClose visionRedClose = new VisionRedClose(hardwareMap);
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
        int zone = visionRedClose.elementDetection(telemetry,new Scalar(255, 0, 0, 1));

        Action drop1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(7,-72))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(7,-20))
                .waitSeconds(0.2)
                .turn(Math.toRadians(90))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-2,-15))
                .waitSeconds(0.2)
                .build();
        Action drop1_2 = drive.actionBuilder(new Pose2d(-2,-15,Math.PI))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(7,-23))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(20,-23))
                .waitSeconds(0.2)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(20,-71.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(110,-71.5))
                .build();
        Action drop2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(7,-72))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(7,-9))
                .waitSeconds(0.2)
                .build();
        Action drop2_2 = drive.actionBuilder(new Pose2d(7,-9,(Math.PI)/2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(7,-71.5))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(110,-71.5))
                .build();
        Action drop3 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0,-20))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(25,-20))
                .build();
        Action drop3_2 = drive.actionBuilder(new Pose2d(25,-20,(Math.PI)/2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(25,-71.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(110,-71.5))
                .build();
        Action wait = drive.actionBuilder(new Pose2d(110,-71.5,(Math.PI)/2))
            .waitSeconds(0.2)
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
                        wrist.toGround(),
                        first,
                        gripper.openLeftGripper(),
                        second,
                        gripper.openRightGripper(),
                wait
                )
        );
    }
}