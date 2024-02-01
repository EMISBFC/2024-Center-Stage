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
    private GripperTest gripperTest;

    public static double p = 0.005, i = 0, d= 0.001;
    public static double f = 0.2;

    public static int target;

    public static int originalArmPos;



    public static double ticks_in_degrees = (double) 360/(28*230*4*231);

    private DcMotorEx arm_motor;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        controller = new PIDController(p, i, d);
        originalArmPos = arm_motor.getCurrentPosition() ;
        gripperTest = new GripperTest(hardwareMap);

    }

    @Override
    public void loop() {
       // target = arm_motor.getCurrentPosition()+5;
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPID(p, i, d);
        gripperTest.handleServo(gamepad1);
        //float target = -gamepad1.left_trigger * 100;
        int armPos = arm_motor.getCurrentPosition();
        if (gamepad1.cross) {

            //arm_motor.setPower(0.1);

           // target = originalArmPos+50;
            target = originalArmPos+Math.abs((int)(originalArmPos*0.2));
        }
        if (gamepad1.triangle) {

            //arm_motor.setPower(0.1);
//            target = originalArmPos+3000;
            target = originalArmPos+Math.abs(originalArmPos*10);
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
