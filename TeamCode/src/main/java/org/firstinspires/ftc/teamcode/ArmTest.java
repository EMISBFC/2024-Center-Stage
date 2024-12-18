package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Position Hold Control", group = "TeleOp")
public class ArmTest extends LinearOpMode {

    // Declare motor
    private DcMotor armMotor;

    // Initialize target position
    private int targetPosition = 0;

    @Override
    public void runOpMode() {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // Reset encoder and set initial mode
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial target position to 0
        targetPosition = 0;

        waitForStart();

        while (opModeIsActive()) {
            // Check for button presses
            if (gamepad1.circle) {
                // Move arm to one side (e.g., -115 encoder counts)
                targetPosition = -115;
            } else if (gamepad1.square) {
                // Move arm to the other side (e.g., 115 encoder counts)
                targetPosition = 115;
            }

            // Continuously hold the target position
            holdPosition(targetPosition);

            // Telemetry for debugging
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // Method to hold the target position
    public void holdPosition(int targetPosition) {
        // Set the target position for the motor
        armMotor.setTargetPosition(targetPosition);

        // Use PID control to maintain position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5); // Power to maintain position
    }
}
