package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Teleop TEST", group="Iterative Opmode")
public class TeleOpTest extends LinearOpMode {

    private GripperTest gripper;


    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {
            gripper = new GripperTest(hardwareMap);
            gripper.handleServo(gamepad1);
        }
    }
}
