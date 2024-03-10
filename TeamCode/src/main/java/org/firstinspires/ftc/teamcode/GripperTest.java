package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperTest {
    private Servo leftGripper;
    private Servo rightGripper;
    boolean leftOpen = false;
    boolean rightOpen = false;
    boolean squareLock = false;
    boolean circleLock = false;




    public GripperTest(HardwareMap hardwareMap){
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad) throws InterruptedException {

        if(gamepad.circle && !circleLock && leftOpen){ // end me , ty u/4106Thumbs
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(0.30);
            circleLock = true;
            leftOpen = false;
        }
        else if(gamepad.circle && !circleLock && !leftOpen){
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(0.50);
            circleLock = true;
            leftOpen = true;
        }
        else if(!gamepad.circle && circleLock) circleLock = false;

        if(gamepad.square && !squareLock && rightOpen){ // end me , ty u/4106Thumbs
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.48);
            squareLock = true;
            rightOpen = false;
        }
        else if(gamepad.square && !squareLock && !rightOpen){
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.23);
            squareLock = true;
            rightOpen = true;
        }
        else if(!gamepad.square && squareLock) squareLock = false;

        if (gamepad.triangle) {
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.23);
            rightOpen = true;
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(0.50);
            leftOpen = true;
        }
        if(gamepad.cross){
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.48);
            rightOpen = false;
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(0.25);
            leftOpen = false;
        }
    }
    public void openLeft(){
//        leftGripper.setDirection(Servo.Direction.FORWARD);
//        leftGripper.setPosition(0.3);
        leftOpen = true;
    }
    public void openRight(){
//        rightGripper.setDirection(Servo.Direction.REVERSE);
//        rightGripper.setPosition(0.75);
        rightOpen = true;
    }
}
