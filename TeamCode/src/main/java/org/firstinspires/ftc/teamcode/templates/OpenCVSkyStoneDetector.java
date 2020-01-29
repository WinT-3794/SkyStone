package org.firstinspires.ftc.teamcode.templates;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.OpenCVHelper;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public abstract class OpenCVSkyStoneDetector extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    OpenCvCamera phoneCam;

    protected void initOpenCV () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new OpenCVHelper());
        phoneCam.startStreaming(Utilities.OCV_ROWS, Utilities.OCV_COLS, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    protected int readOpenCV(int a) {
        int val = -1;
        switch (a){
            case 1:
                val = valLeft;
                break;
            case 2:
                val = valMid;
                break;
            case 3:
                val = valRight;
        }

        return val;
    }


}