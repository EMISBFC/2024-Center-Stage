package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo leftGripper;
    private Servo rightGripper;
    boolean leftOpen = false;
    boolean rightOpen = false;
    boolean squareLock = false;
    boolean circleLock = false;

    public Gripper(HardwareMap hardwareMap){
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad) {

        if(gamepad.circle && !circleLock && leftOpen){ // end me , ty u/4106Thumbs
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(Constants.leftGripperClose);
            circleLock = true;
            leftOpen = false;
        }
        else if(gamepad.circle && !circleLock && !leftOpen){
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(Constants.leftGripperOpen);
            circleLock = true;
            leftOpen = true;
        }
        else if(!gamepad.circle && circleLock) circleLock = false;

        if(gamepad.square && !squareLock && rightOpen){ // end me , ty u/4106Thumbs
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(Constants.rightGripperClose);
            squareLock = true;
            rightOpen = false;
        }
        else if(gamepad.square && !squareLock && !rightOpen){
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(Constants.rightGripperOpen);
            squareLock = true;
            rightOpen = true;
        }
        else if(!gamepad.square && squareLock) squareLock = false;

        if (gamepad.triangle) {
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(Constants.rightGripperOpen);
            rightOpen = true;
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(Constants.leftGripperOpen);
            leftOpen = true;
        }
        if(gamepad.cross){
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(Constants.rightGripperClose);
            rightOpen = false;
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(Constants.leftGripperClose);
            leftOpen = false;
        }
    }
    public void open(){
        rightOpen = true;
        leftOpen = true;
    }
}
