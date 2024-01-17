package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Elevator {
    private Motor ml;
    private Motor mr;

    public Elevator(HardwareMap hardwareMap) {
        ml = new Motor(hardwareMap, "ml");
        mr = new Motor(hardwareMap, "mr");

        //depends on mechanics build
        //ml.setInverted(true);
        //mr.setInverted(true);
    }

    public void upMove ( ) {
        if ( ml.getInverted()){
            ml.setInverted(false);
            mr.setInverted(false);
        }
        ml.set(0.25);
        mr.set(0.25);
    }
    public void downMove ( ) {
        //depends on mechanics build
        if (!ml.getInverted()) {
            ml.setInverted(true);
            mr.setInverted(true);
        }
        ml.set(0.25);
        mr.set(0.25);

    }

}
