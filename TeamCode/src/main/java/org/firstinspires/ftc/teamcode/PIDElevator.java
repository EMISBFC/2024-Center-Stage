package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
@TeleOp
public class PIDElevator extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d= 0;
    public static double f = 0;

    public static int target;



    public static double ticks_in_degrees = (double) 700/180;

    private DcMotorEx arm_motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        controller = new PIDController(p, i, d);


    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
//        float target = -gamepad1.left_trigger * 100;
        int armPos = arm_motor.getCurrentPosition();

        if (gamepad1.cross) {
            arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            //arm_motor.setPower(0.1);
            target = -500;
        }
        if (gamepad1.triangle) {
            arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);
            //arm_motor.setPower(0.1);
            target = -235;
        }
        if(gamepad1.square){
            arm_motor.setPower(1);
        }



        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();




    }

}
