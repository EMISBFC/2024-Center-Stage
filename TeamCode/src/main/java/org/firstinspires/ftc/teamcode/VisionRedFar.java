package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionRedFar {

    //arbitrary values of how much the camera sees
    int camW = 640;
    int camH = 480;
    private OpenCvWebcam camera;
    private int zone;
    private TeamPropDetectionRedFar teamPropDetectionRedFar;

    private String webcamName = "Webcam1";


    public VisionRedFar(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        teamPropDetectionRedFar = new TeamPropDetectionRedFar();

        camera.setPipeline(teamPropDetectionRedFar);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

        public int elementDetection(Telemetry telemetry, Scalar alliance) {
            teamPropDetectionRedFar.setAlliance(alliance);
            zone = teamPropDetectionRedFar.getZone();
            telemetry.addData("Element Zone", zone);
            telemetry.update();
            return zone;
        }






}
