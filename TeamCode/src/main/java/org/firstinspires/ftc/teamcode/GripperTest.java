package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperTest {
    private Servo leftGripper;
    private Servo rightGripper;
    boolean leftOpen = false;
    boolean rightOpen = false;
    boolean triLock = false;
    boolean crossLock = false;




    public GripperTest(HardwareMap hardwareMap){
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad) throws InterruptedException {

        if(gamepad.triangle && !triLock && leftOpen){ // end me , ty u/4106Thumbs
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.5);
            triLock = true;
            leftOpen = false;

        }
        else if(gamepad.triangle && !triLock && !leftOpen){
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.3);
            triLock = true;
            leftOpen = true;

        }
        else if(!gamepad.triangle && triLock) triLock = false;


        if(gamepad.cross && !crossLock && rightOpen){ // end me , ty u/4106Thumbs
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.9);
            crossLock = true;
            rightOpen = false;

        }
        else if(gamepad.cross && !crossLock && !rightOpen){
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.75);
            crossLock = true;
            rightOpen = true;

        }
        else if(!gamepad.cross && crossLock) crossLock = false;
    }
    public void openLeft(){
        leftGripper.setDirection(Servo.Direction.FORWARD);
        leftGripper.setPosition(0.3);
        leftOpen = true;
    }
    public void openRight(){
        rightGripper.setDirection(Servo.Direction.REVERSE);
        rightGripper.setPosition(0.75);
        rightOpen = true;
    }
}
