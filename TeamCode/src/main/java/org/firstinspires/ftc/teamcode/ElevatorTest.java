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
public class ElevatorTest {
    private DcMotor motor1;
    private DcMotor motor2;

    public ElevatorTest(HardwareMap hardwareMap){
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
    }

    public void handleMotors(Gamepad gamepad){
        if(gamepad.left_trigger>0.3){
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1.setPower(gamepad.left_trigger);
            motor2.setDirection(DcMotorSimple.Direction.FORWARD);
            motor2.setPower(gamepad.left_trigger);
        }
        if(gamepad.right_trigger>0.3){
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1.setPower(gamepad.right_trigger);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);
            motor2.setPower(gamepad.right_trigger);
        }
        else if (gamepad.left_trigger<0.3 && gamepad.right_trigger<0.3){
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
}
