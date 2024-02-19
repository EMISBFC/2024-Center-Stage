
package org.firstinspires.ftc.teamcode;
//NOTE TO SELF: STRAFE IN THE X AXIS MOVES HALF THAN YOU'D EXPECT SO J PUT DOUBLE THE DISTANCE :)
// ALSO THE GRIPPERS LEFT IS RIGHT RIGHT IS LEFT :))))))))
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.TeamPropDetection;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ConstantTrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
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
                wristGripper.setPosition(0.65);
                return false;
            }

        }
        public Action toGround(){
            return new ToGround();
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
                leftGripper.setDirection(Servo.Direction.FORWARD);
                leftGripper.setPosition(0.4);
                return false;
            }

        }
        public class CloseLeftGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftGripper.setDirection(Servo.Direction.FORWARD);
                leftGripper.setPosition(0.1);
                return false;
            }

        }
        public class OpenRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.35);
                return false;
            }

        }
        public class CloseRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.05);
                return false;
            }

        }
        public Action openLeftGripper() {
            return new OpenLeftGripper();
        }
        public Action openRightGripper() {
            return new OpenRightGripper();
        }
        public Action closeLeftGripper() {
            return new CloseLeftGripper();
        }
        public Action closeRightGripper() {
            return new CloseRightGripper();
        }




    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(-122, 72, (3*Math.PI)/2);
        Pose2d beginPose = new Pose2d(0, 72, (3*Math.PI)/2);

            Vision vision = new Vision(hardwareMap);
            AGripper gripper = new AGripper(hardwareMap);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            AWrist wrist = new AWrist(hardwareMap);

            waitForStart();


        Action first;
        Action second;
        int zone = vision.elementDetection(telemetry,new Scalar(255, 0, 0, 1));



        Action drop1 = drive.actionBuilder(beginPose)

                .strafeTo(new Vector2d(-102,72))
                .waitSeconds(0.2)
                .lineToY(40)
                .waitSeconds(0.2)
                .build();
        Action drop1_2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.4)
                .lineToY(71.5)
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(210,71.5))
                .build();
        Action drop2 = drive.actionBuilder(beginPose)
                .lineToY(22)
                .waitSeconds(0.2)
                .build();
        Action drop2_2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.4)
                .lineToY(71.5)
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(210,71.5))
                .build();
        Action drop3 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-157,72))
                .waitSeconds(0.2)
                .lineToY(40)
                .build();
        Action drop3_2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.4)
                .lineToY(71.5)
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(210,71.5))
                .build();
        Action park = drive.actionBuilder(drive.pose)
                .waitSeconds(0.4)
                .lineToY(70)
                .waitSeconds(0.2)
                .lineToY(71.5)
                .build();
        Action twoTiles = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(110,72))
                .build();



        if (zone == 3) {
            first = drop3;
        } else if (zone == 2) {
            first = drop2;
        } else {
            first = drop1;
        }

        if (zone == 3) {
            second = drop3_2;
        } else if (zone == 2) {
            second = drop2_2;
        } else {
            second = drop1_2;
        }

        Actions.runBlocking(new SequentialAction(
                        /*gripper.closeLeftGripper(),
                        gripper.closeRightGripper(),
                        wrist.toGround(),
                        first,
                        gripper.openRightGripper(),
                        second,
                        gripper.openLeftGripper(),
                        park*/
                twoTiles
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
