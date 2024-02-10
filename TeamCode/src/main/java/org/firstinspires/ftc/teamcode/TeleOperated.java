package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="ESC Teleop", group="Iterative Opmode")
@Disabled
public class TeleOperated extends LinearOpMode {
    private Arm arm;
    private Chassis chassis;
    private DcMotorController dcMotorController;
    private HardwareMap hardwareMap;
    private Elevator elevator;


    private ServoController servoController;

    public void runOpMode (){
        waitForStart();
        while (opModeIsActive()) {
            chassis = new Chassis(hardwareMap);
            elevator = new Elevator (dcMotorController);
            arm = new Arm(dcMotorController, hardwareMap, servoController);
            arm.handleServo(gamepad1);
        }

    }





}
