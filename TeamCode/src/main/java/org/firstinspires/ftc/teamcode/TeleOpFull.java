package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp(name="Teleop FULL")
public class TeleOpFull extends OpMode {
    private ElevatorTest elevator;

    private RevIMU imu;

    private Chassis chassis;
    private PIDController controller;
    private GripperTest gripper;

    private Wrist wrist;

    public static double p = 0.005, i = 0, d= 0.001;
    public static double f = 0.225;

    public static int target;

    public static int initialArmPos = 2200;
    private static int armPosFloor = 2350;
    private static int armPosHang = 150;
    private static int armPosBackdrop = -800;

    public static double ticks_in_degrees = (double) 360/(28*230*4*231);

    private DcMotorEx arm_motor;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //originalArmPos = arm_motor.getCurrentPosition() ;
        target = initialArmPos;
        controller = new PIDController(p, i, d);

        gripper = new GripperTest(hardwareMap);
        wrist = new Wrist(hardwareMap);
        elevator = new ElevatorTest(hardwareMap);

        imu = new RevIMU(hardwareMap);
        imu.init();

        chassis = new Chassis(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //originalArmPos = arm_motor.getCurrentPosition() ;
        controller = new PIDController(p, i, d);

        gripper.openRight();
        gripper.openLeft();

        wrist.goToFloorPosition();



    }

    @Override
    public void loop() {

        double y = (gamepad1.left_stick_y);
        double x = (-gamepad1.left_stick_x);
        double rx = (-gamepad1.right_stick_x);
        double acc = gamepad1.right_trigger;
        double heading = imu.getRotation2d().getDegrees();
        try {
            gripper.handleServo(gamepad2);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        chassis.fieldCentricDrive(x, y, rx, heading, acc);
//        chassis.robotCentricDrive(x, y, rx, acc);
//        wrist.handleWristServo(gamepad2);
        elevator.handleMotors(gamepad2);

       // target = arm_motor.getCurrentPosition()+5;

        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPID(p, i, d);


        int armPos = arm_motor.getCurrentPosition();

        if (gamepad2.dpad_down) {

            wrist.goToFloorPosition();
            while (target < armPosFloor) {
                target++;
            }
//            for(int i=0;i<2000;i++) {
//                target = initialArmPos -3200 + i;
//            }

        }
        if (gamepad2.right_bumper) {


            while (target != armPosHang) {
                if(target> armPosHang){
                    target--;
                }
                else{
                    target++;
                }
            }
//            for(int i=0;i<2000;i++) {
//                target = initialArmPos -3200 + i;
//            }

        }

        if (gamepad2.dpad_up) {

            //arm_motor.setPower(0.1);
            while (target > armPosBackdrop) {
                target--;
//                System.out.println(target);
            }
//            for(int i=0;i<3700;i++) target = initialArmPos - i;
            wrist.goToBackboardPos();
////          target = originalArmPos+Math.abs(originalArmPos*10);
        }



        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = 0.75*(pid + ff);

        arm_motor.setPower(power);
        try {
            gripper.handleServo(gamepad2);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();




    }

}
