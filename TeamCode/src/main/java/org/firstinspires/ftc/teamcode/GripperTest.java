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
import java.util.concurrent.TimeUnit;
public class GripperTest {
    private Servo leftGripper;
    private Servo rightGripper;
    private boolean leftOpen;
    private boolean rightOpen;
    private double twat;
    String triState;


    public GripperTest(HardwareMap hardwareMap) {
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad) throws InterruptedException {

        if(triState.equals("pressed") && gamepad.triangle){
            triState = "unpressed";
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.5);
            Thread.sleep(50);
        }
        if(triState.equals("unpressed") && gamepad.triangle){
            triState = "pressed";
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.3);
            Thread.sleep(50);

        }

//        if (gamepad.triangle) {
//            switch ((int) (leftGripper.getPosition() * 10)) {
//                case 3:
//                    leftGripper.setPosition(0.5);
//                    leftGripper.setDirection(Servo.Direction.FORWARD);
//                    Thread.sleep(50);
//                    break;
//                case 5:
//                    leftGripper.setPosition(0.3);
//                    leftGripper.setDirection(Servo.Direction.FORWARD);
//                    Thread.sleep(50);
//                    break;
//
//
//            }
//        }



//        if (!leftOpen && gamepad.triangle && !triState) {
//            leftOpen = true;
//            triState = true;
//            leftGripper.setDirection(Servo.Direction.FORWARD);
//            leftGripper.setPosition(0.3);
//            Thread.sleep(50);
//            //  TimeUnit.MILLISECONDS.sleep(200);
//        }
//
//        if (leftOpen && gamepad.triangle && !triState) {
//            leftOpen = false;
//            triState = true;
//            leftGripper.setDirection(Servo.Direction.FORWARD);
//            leftGripper.setPosition(0.5);
//            Thread.sleep(50);
////            TimeUnit.MILLISECONDS.sleep(200);
//        }
//
//        triState = false;
//    }
    }

    public void openLeft(){
        leftGripper.setDirection(Servo.Direction.FORWARD);
        leftGripper.setPosition(0.3);
        triState = "pressed";
    }
    public void openRight(){
        rightGripper.setDirection(Servo.Direction.REVERSE);
        twat = 0.75;
        rightGripper.setPosition(twat);
        rightOpen = true;
    }
}








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

// ?