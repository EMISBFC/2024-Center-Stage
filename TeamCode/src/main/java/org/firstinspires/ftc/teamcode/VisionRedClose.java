package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VisionRedClose {

    //arbitrary values of how much the camera sees
    int camW = 640;
    int camH = 480;
    private OpenCvWebcam camera;
    private int zone;
    private VisionMagic visionMagic;


    private String webcamName = "Webcam1";


    public VisionRedClose(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        visionMagic = new VisionMagic();

        camera.setPipeline(visionMagic);
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
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            int zone = visionMagic.getZone();
            telemetry.addData("Zone ", visionMagic.getZone());
            telemetry.addData("SatZone2 ", visionMagic.getSatZone2());
            telemetry.addData("SatZone3 ", visionMagic.getSatZone3());
            telemetry.addData("Percentage Diff ", visionMagic.getPercentDifference());
            telemetry.update();
            return zone;
        }






}
