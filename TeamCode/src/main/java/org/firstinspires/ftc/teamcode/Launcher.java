package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {

    public CRServo launcherServo;

    public Launcher(HardwareMap hardwareMap) {
        launcherServo = hardwareMap.crservo.get("launcher_servo");
    }

    public void launch(){
        launcherServo.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherServo.setPower(1);
    }
}
