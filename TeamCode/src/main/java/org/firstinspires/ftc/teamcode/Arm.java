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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class Arm  {
    private Servo leftGripper;
    private Servo rightGripper;
    private Servo handJoint;
    private DcMotor middleJoint;
    private Motor leftElevator;
    private Motor rightElevator;

    private double servoControl= 0.05;
    private double ticksPerDeg = 21.94;
    private int ticksperRev = 288;

    private int setTickNumber=30;//fixed number of ticks that will allow the hand to
    // take off from the ground so that we can align it to the rest of the arm before fully turning
    public Arm(DcMotorController dcMotorController, HardwareMap hardwareMap, ServoController servoController){//need to change port number upon build
        leftGripper = new ServoImpl(servoController, 1);
        rightGripper = new ServoImpl(servoController, 1);
        handJoint = new ServoImpl(servoController, 1);
        middleJoint = new DcMotorImpl(dcMotorController, 2);
        leftElevator = new Motor(hardwareMap, "leftElevator");
        rightElevator = new Motor(hardwareMap, "rightElevator");
    }

    public void handleServo(Gamepad gamepad)  {
        if(gamepad.triangle){ // close left
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(0.4);
            //leftGripper.setDirection(Servo.Direction.REVERSE);
        } if (gamepad.cross) { // open left
            leftGripper.setDirection(Servo.Direction.REVERSE);
            leftGripper.setPosition(0.1);

        } if (gamepad.circle) { // close right
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.2);

        } if (gamepad.square) { //open right
            rightGripper.setDirection(Servo.Direction.REVERSE);
            rightGripper.setPosition(0.9);

        }
    }
    public void openRight(){
        rightGripper.setDirection(Servo.Direction.REVERSE);
        rightGripper.setPosition(0.9);
    }
    public void openLeft(){
        rightGripper.setDirection(Servo.Direction.REVERSE);
        rightGripper.setPosition(0.1);
    }

    public void closeRight(){
        rightGripper.setDirection(Servo.Direction.REVERSE);
        rightGripper.setPosition(0.2);
    }
    public void closeLeft(){
        leftGripper.setDirection(Servo.Direction.REVERSE);
        leftGripper.setPosition(0.4);
    }


    public void goToBackboard() { // assume "0 degrees", is 0 from the left, so we want to get to 150 degrees
        double ticksDif = (150*ticksPerDeg) - (middleJoint.getCurrentPosition() % ticksperRev);

        middleJoint.setTargetPosition(setTickNumber);
        middleJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //assume that servo handJoint  pos = 0 is parallel to the ground

        //now when you find out the angle at which the middle joint hits the ground you will
        //know the angle to move to line handJoint up with the middleJoint, for now assume it's 45degrees or 0.25 of 180

        handJoint.setPosition(0.25);


        int backboardTarget = middleJoint.getCurrentPosition() + (int)ticksDif-setTickNumber;
        middleJoint.setTargetPosition(backboardTarget);
        middleJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void goToGround() { // assume "0 degrees", is 0 from the left, so we want to get to 150 degrees
        middleJoint.setDirection(DcMotor.Direction.REVERSE);//moving back

        middleJoint.setTargetPosition(setTickNumber);//move away from backboard
        middleJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        handJoint.setPosition(0);//make parallel with ground

        int ticksDif = middleJoint.getCurrentPosition()%288-setTickNumber;
        middleJoint.setTargetPosition(ticksDif);
        middleJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


}

