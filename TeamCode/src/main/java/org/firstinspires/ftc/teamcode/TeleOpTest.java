package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Teleop TEST", group="Iterative Opmode")
@Disabled
public class TeleOpTest extends OpMode {

    private GripperTest gripper;
    private ElevatorTest elevator;

    private RevIMU imu;

    private Chassis chassis;

    private PIDController controller;
    //private Wrist wrist;

    public static double p = 0.005, i = 0, d= 0.001;
    public static double f = 0.22;

    public static int target;

    public static int originalArmPos;



    public static double ticks_in_degrees = (double) 360/(28*230*4*231);

    private DcMotorEx arm_motor;
//
//    private PIDElevator pidElevator;

//    double x, double y, double rx, double heading, double acc;



//    @Override
//
//    public void runOpMode() {
//        gripper = new GripperTest(hardwareMap);
//
//        imu = new RevIMU(hardwareMap);
//        imu.init();
//
//        chassis = new Chassis(hardwareMap);
//
////        elevator = new ElevatorTest(hardwareMap);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            double y = (gamepad1.right_stick_y);
//            double x = (-gamepad1.right_stick_x);
//            double rx = (-gamepad1.left_stick_x);
//            double acc = gamepad1.right_trigger;
//            double heading = imu.getRotation2d().getDegrees();
//            gripper.handleServo(gamepad2);
//
//
//
//
////            chassis.robotCentricDrive(x, y, rx, acc);
//            chassis.fieldCentricDrive(x, y, rx, heading, acc);
////            elevator.handleMotors(gamepad1);
//
////            pidElevator.init();
////            pidElevator.loop();
//
//        }
//    }

    @Override
    public void init() {
        gripper = new GripperTest(hardwareMap);
        //wrist = new Wrist(hardwareMap);
        elevator = new ElevatorTest(hardwareMap);

        imu = new RevIMU(hardwareMap);
        imu.init();

        chassis = new Chassis(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //originalArmPos = arm_motor.getCurrentPosition() ;
        controller = new PIDController(p, i, d);


    }

    @Override
    public void loop() {
        double y = (gamepad1.right_stick_y);
        double x = (-gamepad1.right_stick_x);
        double rx = (-gamepad1.left_stick_x);
        double acc = gamepad1.right_trigger;
        double heading = imu.getRotation2d().getDegrees();
        try {
            gripper.handleServo(gamepad2);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        chassis.fieldCentricDrive(x, y, rx, heading, acc);
        //wrist.handleWristServo(gamepad2);
        elevator.handleMotors(gamepad2);

        //arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPID(p, i, d);

        //float target = -gamepad1.left_trigger * 100;
        //int armPos = arm_motor.getCurrentPosition();
      //  if (gamepad1.cross) {

            //arm_motor.setPower(0.1);

         //   target = originalArmPos+50;
//            target = originalArmPos+Math.abs((int)(originalArmPos*0.2));
        //}
        //if (gamepad1.triangle) {

            //arm_motor.setPower(0.1);
            //target = originalArmPos+3000;
//            target = originalArmPos+Math.abs(originalArmPos*10);
       // }




        //double pid = controller.calculate(armPos, target);
        //double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        //double power = pid + ff;

        //arm_motor.setPower(power);

        //telemetry.addData("pos", armPos);
        //telemetry.addData("target", target);
        //telemetry.update();

    }
}
