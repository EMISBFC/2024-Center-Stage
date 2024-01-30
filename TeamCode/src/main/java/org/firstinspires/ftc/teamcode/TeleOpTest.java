package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Teleop TEST", group="Iterative Opmode")
public class TeleOpTest extends LinearOpMode {

    private GripperTest gripper;
    private ElevatorTest elevator;

    private Chassis chassis;

    //double x, double y, double rx, double heading, double acc

    @Override
    public void runOpMode() {
//        gripper = new GripperTest(hardwareMap);

        chassis = new Chassis(hardwareMap);

        elevator = new ElevatorTest(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            double y = (gamepad1.right_stick_y);
            double x = (-gamepad1.right_stick_x);
            double rx = (-gamepad1.left_stick_x);
            double acc = gamepad1.right_trigger;
//            gripper.handleServo(gamepad2);


            chassis.robotCentricDrive(x, y, rx, acc);
            elevator.handleMotors(gamepad1);

        }
    }
}
