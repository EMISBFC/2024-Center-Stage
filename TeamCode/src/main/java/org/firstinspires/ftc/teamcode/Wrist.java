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
public class Wrist {

    private Servo wristGripper;

    public Wrist(HardwareMap hardwareMap){
        wristGripper = hardwareMap.servo.get("wrist_gripper");
    }

    public void handleWristServo(Gamepad gamepad){
        if(gamepad.right_trigger>0){
            wristGripper.setDirection(Servo.Direction.FORWARD);
            wristGripper.setPosition(0.9);
        }
        if (gamepad.left_trigger>0){
            wristGripper.setDirection(Servo.Direction.FORWARD);
            wristGripper.setPosition(0.5);
        }
    }

}
