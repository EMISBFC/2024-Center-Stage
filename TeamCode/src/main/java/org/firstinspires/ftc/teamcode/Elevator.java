package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private DcMotor motor1;
    private DcMotor motor2;

    public Elevator(HardwareMap hardwareMap){
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