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



    public GripperTest(HardwareMap hardwareMap){
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad)  {

        if(gamepad.triangle){ // close left
                leftGripper.setDirection(Servo.Direction.FORWARD);
                leftGripper.setPosition(0.95);


            //leftGripper.setDirection(Servo.Direction.REVERSE);
        } if (gamepad.cross) { // open left
            leftGripper.setDirection(Servo.Direction.FORWARD);
            leftGripper.setPosition(0.25);

        } if (gamepad.circle) { // close right
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.95);
        } if (gamepad.square) { //open right
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.25);

        } else {
//            leftGripper.setPosition(0);
//            rightGripper.setPosition(0);
//            leftGripper.setPosition(leftGripper.getPosition());
//            rightGripper.setPosition(rightGripper.getPosition());
        }


    }
    public void openLeft(){
        leftGripper.setDirection(Servo.Direction.FORWARD);
        leftGripper.setPosition(0.25);
    }
    public void openRight(){
        leftGripper.setDirection(Servo.Direction.REVERSE);
        leftGripper.setPosition(0.25);
    }
}
