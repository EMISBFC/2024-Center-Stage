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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class AutoMovement {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private double tickPerRev = 537.7;
    private double mPerRev = 0.30159289474;
    private double tileLength=0.6096;//in meters
    public AutoMovement(DcMotorController dcMotorController){
        fl = new DcMotorImpl(dcMotorController, 1);
        fr = new DcMotorImpl(dcMotorController, 2);
        bl = new DcMotorImpl(dcMotorController, 3);
        br = new DcMotorImpl(dcMotorController, 4);
    }
    public double metersToTicks(double meters){
        return (meters/mPerRev)*tickPerRev;
    }
    public void moveInTilesUnitForward(double tiles){
        double targetPosition = metersToTicks(tiles*tileLength);
        boolean needToInvert = false;
        if (targetPosition<0){//assume motors go
            fl.setDirection(DcMotor.Direction.REVERSE);
            fr.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            br.setDirection(DcMotor.Direction.REVERSE);
            targetPosition = -targetPosition;
            needToInvert = true;
        }

        fl.setTargetPosition((int)(targetPosition));
        fr.setTargetPosition((int)(targetPosition));
        bl.setTargetPosition((int)(targetPosition));
        br.setTargetPosition((int)(targetPosition));

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(needToInvert){
            fl.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.FORWARD);
            bl.setDirection(DcMotor.Direction.FORWARD);
            br.setDirection(DcMotor.Direction.FORWARD);
        }
    }


}