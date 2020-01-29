package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCVHelper extends OpenCvPipeline
{
    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    public static int valMid = -1;
    public static int valLeft = -1;
    public static int valRight = -1;

    private OpenCVStages stageToRenderToViewport = OpenCVStages.detection;
    private OpenCVStages[] stages = OpenCVStages.values();

    @Override
    public void onViewportTapped()
    {
        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);

        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);

        double[] pixMid = thresholdMat.get((int)(input.rows()* Utilities.MID_POS[1]), (int)(input.cols()* Utilities.MID_POS[0]));
        valMid = (int)pixMid[0];

        double[] pixLeft = thresholdMat.get((int)(input.rows()* Utilities.LEFT_POS[1]), (int)(input.cols()* Utilities.LEFT_POS[0]));
        valLeft = (int)pixLeft[0];

        double[] pixRight = thresholdMat.get((int)(input.rows()* Utilities.RIGHT_POS[1]), (int)(input.cols()* Utilities.RIGHT_POS[0]));
        valRight = (int)pixRight[0];

        Point pointMid = new Point((int)(input.cols()* Utilities.MID_POS[0]), (int)(input.rows()* Utilities.MID_POS[1]));
        Point pointLeft = new Point((int)(input.cols()* Utilities.LEFT_POS[0]), (int)(input.rows()* Utilities.LEFT_POS[1]));
        Point pointRight = new Point((int)(input.cols()* Utilities.RIGHT_POS[0]), (int)(input.rows()* Utilities.RIGHT_POS[1]));

        Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );
        Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );
        Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );

        Imgproc.rectangle(
                all,
                new Point(
                        input.cols()*(Utilities.LEFT_POS[0]-Utilities.RECT_WIDTH/2),
                        input.rows()*(Utilities.LEFT_POS[1]-Utilities.RECT_HEIGHT/2)),
                new Point(
                        input.cols()*(Utilities.LEFT_POS[0]+Utilities.RECT_WIDTH/2),
                        input.rows()*(Utilities.LEFT_POS[1]+Utilities.RECT_HEIGHT/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(
                all,
                new Point(
                        input.cols()*(Utilities.MID_POS[0]-Utilities.RECT_WIDTH/2),
                        input.rows()*(Utilities.MID_POS[1]-Utilities.RECT_HEIGHT/2)),
                new Point(
                        input.cols()*(Utilities.MID_POS[0]+Utilities.RECT_WIDTH/2),
                        input.rows()*(Utilities.MID_POS[1]+Utilities.RECT_HEIGHT/2)),
                new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(
                all,
                new Point(
                        input.cols()*(Utilities.RIGHT_POS[0]-Utilities.RECT_WIDTH/2),
                        input.rows()*(Utilities.RIGHT_POS[1]-Utilities.RECT_HEIGHT/2)),
                new Point(
                        input.cols()*(Utilities.RIGHT_POS[0]+Utilities.RECT_WIDTH/2),
                        input.rows()*(Utilities.RIGHT_POS[1]+Utilities.RECT_HEIGHT/2)),
                new Scalar(0, 255, 0), 3);

        switch (stageToRenderToViewport)
        {
            case THRESHOLD:
            {
                return thresholdMat;
            }

            case detection:
            {
                return all;
            }

            default:
            {
                return input;
            }
        }
    }

}