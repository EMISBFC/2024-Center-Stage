package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "IterativeOpmode")
public class Autonomous extends LinearOpMode {




    private Vision vision;



    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap);
        int elementZone = vision.elementDetection(telemetry);//1 if left, 2 if center, 3 if right
        waitForStart();
        while (opModeIsActive()){
            if(elementZone==1){

            }

        }



    }

}
