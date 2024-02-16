package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

public class OldElevatorCode {
    private DcMotor ml;
    private DcMotor mr;
    private double tickPerRev = 28;
    private double mPerRev = 0.01;//random number to fix later
    private int currentLevel = 0;
    private double  level0Height = 0.15; //in meters, NEED TO FIND ACTUAL VALUE WHEN BUILT
    private double level1Height = 0.4 - level0Height;
    private double level2Height = 0.6 - level0Height;

    public OldElevatorCode(DcMotorController dcMotorController) {
        ml = new DcMotorImpl(dcMotorController, 1);
        mr = new DcMotorImpl(dcMotorController, 2);

        //depends on mechanics build
        //ml.setInverted(true);
        //mr.setInverted(true);
    }
    public void level1 () {
        if (currentLevel == 0) {//if it fails then we have to make the driver click the other level and then try again also big bad if we go to level 2 twice and go over

            ml.setTargetPosition((int)((level1Height/mPerRev)*tickPerRev));
            mr.setTargetPosition((int)((level1Height/mPerRev)*tickPerRev));

            ml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(currentLevel==2){
            ml.setDirection(DcMotor.Direction.REVERSE);
            mr.setDirection(DcMotor.Direction.REVERSE);
            ml.setTargetPosition((int)((0.2/mPerRev)*tickPerRev));
            mr.setTargetPosition((int)((0.2/mPerRev)*tickPerRev));

            ml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ml.setDirection(DcMotor.Direction.FORWARD);
            mr.setDirection(DcMotor.Direction.FORWARD);
        }
    }
    public void level0(){
        if (currentLevel == 1) {//if it fails then we have to make the driver click the other level and then try again also big bad if we go to level 2 twice and go over
            ml.setDirection(DcMotor.Direction.REVERSE);
            mr.setDirection(DcMotor.Direction.REVERSE);
            ml.setTargetPosition((int)((level1Height/mPerRev)*tickPerRev));
            mr.setTargetPosition((int)((level1Height/mPerRev)*tickPerRev));

            ml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ml.setDirection(DcMotor.Direction.FORWARD);
            mr.setDirection(DcMotor.Direction.FORWARD);
        }
        else if(currentLevel==2){
            ml.setDirection(DcMotor.Direction.REVERSE);
            mr.setDirection(DcMotor.Direction.REVERSE);
            ml.setTargetPosition((int)((level2Height/mPerRev)*tickPerRev));
            mr.setTargetPosition((int)((level2Height/mPerRev)*tickPerRev));

            ml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ml.setDirection(DcMotor.Direction.FORWARD);
            mr.setDirection(DcMotor.Direction.FORWARD);
        }

    }

    public void level2() {
        if (currentLevel == 0) {//if it fails then we have to make the driver click the other level and then try again also big bad if we go to level 2 twice and go over

            ml.setTargetPosition((int)((level2Height / mPerRev) * tickPerRev));
            mr.setTargetPosition((int)((level2Height / mPerRev) * tickPerRev));
            ml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (currentLevel == 1) {
            ml.setTargetPosition((int)((0.2 / mPerRev) * tickPerRev));
            mr.setTargetPosition((int)((0.2 / mPerRev) * tickPerRev));

            ml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    /*
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
    */


}