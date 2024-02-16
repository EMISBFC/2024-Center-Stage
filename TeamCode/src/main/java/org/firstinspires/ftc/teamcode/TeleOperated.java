package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name="ESC Teleop", group="Iterative Opmode")
@Disabled
public class TeleOperated extends LinearOpMode {
    private OldArmCode oldArmCode;
    private Chassis chassis;
    private DcMotorController dcMotorController;
    private HardwareMap hardwareMap;
    private OldElevatorCode oldElevatorCode;


    private ServoController servoController;

    public void runOpMode (){
        waitForStart();
        while (opModeIsActive()) {
            chassis = new Chassis(hardwareMap);
            oldElevatorCode = new OldElevatorCode(dcMotorController);
            oldArmCode = new OldArmCode(dcMotorController, hardwareMap, servoController);
            oldArmCode.handleServo(gamepad1);
        }

    }





}
