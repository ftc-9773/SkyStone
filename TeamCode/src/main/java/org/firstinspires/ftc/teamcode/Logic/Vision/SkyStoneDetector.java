package org.firstinspires.ftc.teamcode.Logic.Vision;

import android.util.Log;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.*;

import java.util.ArrayList;
import java.util.List;

//For detecting skystones.
//call init, then enable, the getPosition to get the position. To stop looking at frames. call disable.
public class SkyStoneDetector extends OpenCVPipeline {
    static final String TAG = "ftc9773skyStoneDetector";
    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat input = new Mat();
    Mat greyscale = new Mat();
    double midval;
    double leftval;
    List<MatOfPoint> contoursList = new ArrayList();
    public enum skyPositions {unknown, left, mid, right}
    skyPositions position = skyPositions.unknown;

    int threshold;
    // [x,y] in terms of %width and %height
    double[] mid = new double[2];
    double[] left = new double[2];
    double[] right = new double[2];
    int width, height;
    SafeJsonReader reader;


    public SkyStoneDetector(){
        this(false);
    }

    public SkyStoneDetector(boolean isblue){
        super();
        reader = new SafeJsonReader("VisionThresholds");

        threshold = reader.getInt("detectionThreshold");
        if (isblue){
            mid[0] = reader.getDouble("bluemidx");
            mid[1] = reader.getDouble("bluemidy");
            left[0] = reader.getDouble("blueleftx");
            left[1] = reader.getDouble("bluelefty");
            right[0] = reader.getDouble("bluerightx");
            right[1] = reader.getDouble("bluerighty");

        } else {
            mid[0] = reader.getDouble("midx");
            mid[1] = reader.getDouble("midy");
            left[0] = reader.getDouble("leftx");
            left[1] = reader.getDouble("lefty");
            right[0] = reader.getDouble("rightx");
            right[1] = reader.getDouble("righty");
            }
    }

    public skyPositions getPosition(){
        return position;
    }

    //Called each new frame.
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame){
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat input;
        input = inputFrame.rgba();
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, greyscale, 1);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2); //Flatten into 2d mat which is blue difference.

        Imgproc.threshold(yCbCrChan2Mat, thresholdMat,threshold,255, Imgproc.THRESH_BINARY_INV);
        width = yCbCrChan2Mat.cols();
        height = yCbCrChan2Mat.rows();

        Log.d(TAG, "width " + width);
        Log.d(TAG, "height " + height);
        Log.d(TAG, "thresholdmatdims " + yCbCrChan2Mat.dims());

        int y = (int) (mid[1] * height);
        int midx = (int) (mid[0] * width);
        int leftx = (int) (left[0] *width);
        int rightx = (int) (right[0] * width);


        leftval =  yCbCrChan2Mat.get(y, leftx)[0];
        midval =  yCbCrChan2Mat.get(y, midx)[0];

        Log.d(TAG, " len " + yCbCrChan2Mat.get(y, leftx).length);
        Log.d(TAG, " len " + yCbCrChan2Mat.get(y, midx).length);

        if (leftval > 100) {
            position = skyPositions.left;
        } else if (midval > 100) {
            position = skyPositions.mid;
        } else {
            position = skyPositions.right;
        }
        Log.d(TAG, "left " + leftval);
        Log.d(TAG, "mid " + midval);

//        Log.d(TAG, "right " + yCbCrChan2Mat.get(rightx, y)[0]); DOESN"T WORK FOR SOME REASON!!

        Point midPoint   = new Point(midx, y);
        Point rightPoint = new Point(rightx, y);
        Point leftPoint  = new Point(leftx, y);

        Imgproc.circle(yCbCrChan2Mat, midPoint, 5, new Scalar(255, 0, 0), 1);
        Imgproc.circle(yCbCrChan2Mat, leftPoint, 5, new Scalar(255, 0, 0), 1);
        Imgproc.circle(yCbCrChan2Mat, rightPoint, 5, new Scalar(255, 0, 0), 1);

        return yCbCrChan2Mat;
    }


}
