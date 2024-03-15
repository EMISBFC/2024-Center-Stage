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

@Autonomous(name = "AABlueFar", group = "Autonomous")
public class AABlueFar extends LinearOpMode {
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
        Pose2d beginPose = new Pose2d(-110, 72, (3*Math.PI)/2);

        VisionBlueFar visionBlueFar = new VisionBlueFar(hardwareMap);
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
        int zone = visionBlueFar.elementDetection(telemetry,new Scalar(0, 0, 255, 1));

        Action drop1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-120,72))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-120,20))
                .turn(Math.toRadians(90))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-108,15))
                .build();
        Action drop1_2 = drive.actionBuilder(new Pose2d(-108,15,0))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-120,20))
                .waitSeconds(0.2)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-120,65))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(65,65))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(65,71.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(110,71.5))
                .waitSeconds(0.5)
                .build();
        Action drop2 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-117,72))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-117,9))
                .waitSeconds(0.2)
                .build();
        Action drop2_2 = drive.actionBuilder(new Pose2d(-117,9,(3*Math.PI)/2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-117,65))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(65,65))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(65,71.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(110,71.5))
                .build();
        Action drop3 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-133,72))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-133,20))
                .waitSeconds(0.2)
                .build();
        Action drop3_2 = drive.actionBuilder(new Pose2d(-133,20,(3*Math.PI)/2))
                .waitSeconds(0.4)
                .strafeTo(new Vector2d(-153,65))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(65,65))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(65,71.5))
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
                        gripper.openLeftGripper(),
                        second,
                        gripper.openRightGripper(),
                        wait,
                        wrist.toUp(),
                        wait
                )
        );
    }
}
