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

    private Elevator elevator;
    private RevIMU imu;
    private Chassis chassis;
    private PIDController controller;
    private Gripper gripper;
    private Wrist wrist;
    private Launcher launcher;
    public static int target;
    private DcMotorEx arm_motor;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(Constants.p, Constants.i, Constants.d);
        imu = new RevIMU(hardwareMap);

        chassis = new Chassis(hardwareMap);
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);
        elevator = new Elevator(hardwareMap);
        launcher = new Launcher(hardwareMap);

        imu.init();
        target = Constants.initialArmPos;
        gripper.open();
        wrist.goToFloorPosition();

    }

    @Override
    public void loop() {

        double y = (gamepad1.left_stick_y);
        double x = (-gamepad1.left_stick_x);
        double rx = (-gamepad1.right_stick_x);
        double acc = gamepad1.right_trigger;
        double heading = imu.getRotation2d().getDegrees();

        chassis.fieldCentricDrive(x, y, rx, heading, acc);
        elevator.handleMotors(gamepad2);
        gripper.handleServo(gamepad2);

        if(gamepad1.triangle) launcher.launch();
        else launcher.launcherServo.setPower(0);

        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setPID(Constants.p, Constants.i, Constants.d);

        int armPos = arm_motor.getCurrentPosition();

        if (gamepad2.dpad_down) {
            wrist.goToFloorPosition();
            while (target < Constants.armPosFloor) target++;
        }
        if (gamepad2.right_bumper) {
            while (target != Constants.armPosHang) {
                if(target> Constants.armPosHang)target--;
                else target++;
            }
        }
        if (gamepad2.dpad_up) {
            while (target > Constants.armPosBackdrop) target--;
            wrist.goToBackboardPos();
        }
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/Constants.ticks_in_degrees)) * Constants.f;
        double power = 0.75*(pid + ff);
        arm_motor.setPower(power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();

    }
}
