package org.firstinspires.ftc.teamcode;
//NOTE TO SELF: STRAFE IN THE X AXIS MOVES HALF THAN YOU'D EXPECT SO J PUT DOUBLE THE DISTANCE :)

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.TeamPropDetection;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ConstantTrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
import org.opencv.core.Scalar;


@Autonomous(name = "AABlueClose", group = "Autonomous")
public class AABlueClose extends LinearOpMode {
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
                leftGripper.setPosition(0.3);
                return false;
            }

        }
        public class OpenRightGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.75);
                return false;
            }

        }
        public Action openLeftGripper() {
            return new OpenLeftGripper();
        }
        public Action openRightGripper() {
            return new OpenRightGripper();
        }



    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 72, (3*Math.PI)/2);

            Vision vision = new Vision(hardwareMap);
            AGripper gripper = new AGripper(hardwareMap);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


        Action first;
        Action second;
        Action  park;
        int zone = vision.elementDetection(telemetry,new Scalar(255, 0, 0, 1));



        Action drop1 = drive.actionBuilder(beginPose)
                .lineToY(40)
                .waitSeconds(0.2)
                .turn((Math.PI)/2)
                .waitSeconds(0.2)
                .build();
        Action drop1_2 = drive.actionBuilder(drive.pose)
                .turn((3*Math.PI)/2)
                .waitSeconds(0.2)
                .setTangent((3*(Math.PI))/2)
                .lineToY(72)
                .build();
        Action drop2 = drive.actionBuilder(beginPose)
                .lineToY(20)
                .waitSeconds(0.2)
                .build();
        Action drop2_2 = drive.actionBuilder(drive.pose)
                .lineToY(72)
                .build();
        Action drop3 = drive.actionBuilder(beginPose)
                .lineToY(40)
                .waitSeconds(0.2)
                .turn(-(Math.PI)/2)
                .waitSeconds(0.2)
                .build();
        Action drop3_2 = drive.actionBuilder(drive.pose)
                .turn((Math.PI)/2)
                .waitSeconds(0.2)
                .setTangent((3*(Math.PI))/2)
                .lineToY(72)
                .build();

        park = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(60,72))
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
                        drop1,
                        gripper.openLeftGripper(),
                        drop1_2,
                        park,
                        gripper.openRightGripper()
                )

                //gripper.openGripper()

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
