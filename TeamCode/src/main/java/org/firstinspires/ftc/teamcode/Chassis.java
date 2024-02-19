package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class Chassis {
    private Motor fl;
    private Motor fr;
    private Motor bl;
    private Motor br;

    MecanumDrive mecanum;
    public Chassis(HardwareMap hardwareMap){
//		fl = hardwareMap.get(DcMotorEx.class, "fl");
//		fr = hardwareMap.get(DcMotorEx.class, "fr");
//		bl = hardwareMap.get(DcMotorEx.class, "bl");
//		br = hardwareMap.get(DcMotorEx.class, "br");
//		vs = hardwareMap.get(VoltageSensor.class, "fl");

        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");


//		br.setDirection(DcMotorEx.Direction.REVERSE);
//		fr.setDirection(DcMotorEx.Direction.REVERSE);

//		br.setInverted(true);
//		fr.setInverted(true);

        mecanum = new MecanumDrive(fl, fr, bl, br);

    }



    public void fieldCentricDrive(double x, double y, double rx, double heading, double acc){
        if(acc>1) {
            mecanum.driveFieldCentric(x * 0.75, y * 0.75, rx * 0.75, heading);
        }
        else{
            mecanum.driveFieldCentric(x * 1, y * 1, rx * 1, heading);
        }
    }
    public void robotCentricDrive(double x, double y, double rx, double acc){
        if(acc>0.1){
            mecanum.driveRobotCentric(x*0.75, y*0.75, rx*0.75);
        }
        else{
            mecanum.driveRobotCentric(x*1, y*1, rx*1);
        }
    }
}
