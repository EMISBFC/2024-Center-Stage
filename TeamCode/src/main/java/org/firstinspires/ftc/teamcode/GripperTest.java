package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
public class GripperTest {
    private Servo leftGripper;
    private Servo rightGripper;
    private boolean leftOpen;
    private boolean rightOpen;
    private double twat;



    public GripperTest(HardwareMap hardwareMap){
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad) throws InterruptedException {


//        if(gamepad.triangle){ // close left
//                leftGripper.setDirection(Servo.Direction.FORWARD);
//                leftGripper.setPosition(0.5);
//            //leftGripper.setDirection(Servo.Direction.REVERSE);
//        } if (gamepad.cross) { // open left
//            leftGripper.setDirection(Servo.Direction.FORWARD);
//            leftGripper.setPosition(0.3);
//
//        } if (gamepad.circle) { // close right
//            rightGripper.setDirection(Servo.Direction.REVERSE);
//            rightGripper.setPosition(0.9);
//        } if (gamepad.square) { //open right
//            rightGripper.setDirection(Servo.Direction.REVERSE);
//            rightGripper.setPosition(0.75);
//
//        } else {
////            leftGripper.setPosition(0);
////            rightGripper.setPosition(0);
////            leftGripper.setPosition(leftGripper.getPosition());
////            rightGripper.setPosition(rightGripper.getPosition());
//        }

        if(gamepad.triangle){
            if(gamepad.triangle && leftGripper.getPosition()==0.3 && leftOpen){
                leftOpen = false;
                leftGripper.setDirection(Servo.Direction.FORWARD);
                leftGripper.setPosition(0.5);
                Thread.sleep(500);
            }
            if(gamepad.triangle && leftGripper.getPosition()==0.5 && !leftOpen){
                leftOpen = true;
                leftGripper.setDirection(Servo.Direction.FORWARD);
                leftGripper.setPosition(0.3);
                Thread.sleep(500);
            }

            //leftGripper.setDirection(Servo.Direction.REVERSE);
        }

        if (gamepad.cross) {
            if(gamepad.cross && rightGripper.getPosition()==0.75 && rightOpen){
                rightOpen = false;
                rightGripper.setDirection(Servo.Direction.REVERSE);
                twat = 0.9;
//                rightGripper.setPosition(0.9);
//                Thread.sleep(100);
            }
            if(gamepad.cross && rightGripper.getPosition()==0.9 && !rightOpen){
                rightOpen = true;
                rightGripper.setDirection(Servo.Direction.REVERSE);
                twat = 0.75;
//                rightGripper.setPosition(0.75);
//                Thread.sleep(100);

            }
            rightGripper.setPosition(twat);
            Thread.sleep(100);
        }
    }
    public void openLeft(){
        leftGripper.setDirection(Servo.Direction.FORWARD);
        leftGripper.setPosition(0.3);
        leftOpen = true;
    }
    public void openRight(){
        rightGripper.setDirection(Servo.Direction.REVERSE);
        twat = 0.75;
        rightGripper.setPosition(twat);
        rightOpen = true;
    }
}
