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
    private int counterLeft=1;
    private int counterRight=1;
    private Servo leftGripper;
    private Servo rightGripper;

    public GripperTest(HardwareMap hardwareMap){
        leftGripper = hardwareMap.servo.get("leftGripper");
        rightGripper = hardwareMap.servo.get("rightGripper");
    }

    public void handleServo(Gamepad gamepad)  {

        if(gamepad.square){ // close left
            if(counterLeft%2!=0){
                leftGripper.setDirection(Servo.Direction.REVERSE);
                leftGripper.setPosition(0.1);
            }
            else{
                leftGripper.setDirection(Servo.Direction.REVERSE);
                leftGripper.setPosition(0.4);
            }
            counterLeft++;

        } if (gamepad.circle) { // close right
            if (counterRight % 2 != 0) {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.9);
            } else {
                rightGripper.setDirection(Servo.Direction.REVERSE);
                rightGripper.setPosition(0.2);
            }
            counterRight++;
        }

    }
}
