package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Horz {
    private Motor horz;

    public Horz(HardwareMap hardwareMap){
        horz = new Motor(hardwareMap, "horz");

    }
    public void handleHorz(Gamepad gamepad){
        if(gamepad.left_trigger>0.3){
            horz.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            horz.motor.setPower(0.65);
        }
        if(gamepad.right_trigger>0.3){
            horz.motor.setDirection(DcMotorSimple.Direction.FORWARD);
            horz.motor.setPower(0.65);
        }
        else if (gamepad.left_trigger<0.3 && gamepad.right_trigger<0.3){
            horz.motor.setPower(0);
        }
    }
}
