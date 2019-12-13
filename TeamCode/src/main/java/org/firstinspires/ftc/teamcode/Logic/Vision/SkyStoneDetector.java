package org.firstinspires.ftc.teamcode.Logic.Vision;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.*;

public class SkyStoneDetector implements CameraBridgeViewBase.CvCameraViewListener2 {
    static {
        boolean magic = OpenCVLoader.initDebug();
        Log.d("ftc9773skyStoneDetector", "");
    }
    protected JavaCameraView cameraView;
    protected Context context;
    private Mat rgba = new Mat();
    private Mat gray = new Mat();
    private Mat res = new Mat();
    private boolean initStarted = false;
    private boolean inited = false;

    static final String TAG = "ftc9773skyStoneDetector";
    Mat blurred ;
    Mat unblurred ;
    Mat filtered ;
    Boolean leftIsBlack = null, rightIsBlack = null;
    int totalWidth, totalHeight;

    // needs to be tunable
    double [] relBoxOne = {1, 1, 1, 1}; //x, y, w, h
    double [] relBoxTwo = {1, 1, 1, 1};


    double minDetectionThreshold;
    double onVal = 120.0;
    double leftCorrectPercent, rightCorrectPercent;
    final static Scalar blue = new Scalar(0,0,255);
    final static Scalar red = new Scalar(255, 0,0);

    boolean returnMat;

    skyPositions position = skyPositions.unknown;

    //Generated
    private Mat blur0Output = new Mat();
    private Mat hsvThresholdOutput = new Mat();

    public SkyStoneDetector(){
        super();
        SafeJsonReader reader = new SafeJsonReader("VisionThresholds");
        minDetectionThreshold = reader.getDouble("minDetectionThreshold", 0.1);
        // get boxOne Pos
        relBoxOne[0] = reader.getDouble("BoxOneTopLeftX",.1818);
        relBoxOne[1] = reader.getDouble("BoxOneTopLeftY",0.4718137255);
        relBoxOne[2] = reader.getDouble("BoxOneWidth",.3880);
        relBoxOne[3] = reader.getDouble("BoxOneLength",0.6050857843);
        // get boxTwo Pos
        relBoxTwo[0] = reader.getDouble("BoxTwoTopLeftX",0.7761437908);
        relBoxTwo[1] = reader.getDouble("BoxTwoTopLeftY",0.5208333333);
        relBoxTwo[2] = reader.getDouble("BoxTwoWidth",0.9803921569);
        relBoxTwo[3] = reader.getDouble("BoxTwoLength",0.6740196078);

        returnMat = reader.getBoolean("returnMat", false);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Log.d(TAG, "input_dims " + inputFrame.toString());
        return processFrame(inputFrame.rgba());
    }

    /***
     * Called each frame. Updates position using two filters.
     */
    public Mat processFrame(Mat input) {
        // Step 1 Blur:
        Mat blur0Input = input;
        double blur0Radius = 9.90990990990991;
        gaussBlur(blur0Input, blur0Radius, blur0Output);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blur0Output;
        double[] hsvThresholdHue = {6.474820143884892, 43.63636363636364};
        double[] hsvThresholdSaturation = {149.05575539568346, 255.0};
        double[] hsvThresholdValue = {130.71043165467626, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);


        totalWidth = input.width();
        totalHeight = input.height();
        Log.d(TAG, "TotalWidth " + totalWidth);
        Log.d(TAG, "TotalHeight " + totalHeight);
        // find the actual size of the bounding box.
        //boxOneBounds = scaleArray(relBoxOne, totalWidth,totalHeight);
        //boxTwoBounds = scaleArray(relBoxTwo, totalWidth, totalHeight);

//
//        int widthOne = boxOneBounds[1] - boxOneBounds[0];
//        int heightOne = boxOneBounds[3] - boxOneBounds[2];
        double X1 = relBoxOne[0] * totalWidth;
        double Y1 = relBoxOne[1] * totalHeight;
        double relwidth1 = relBoxOne[2] * totalWidth;
        double relheight1 = relBoxOne[3] * totalHeight;

        Log.d(TAG, "X1,Y1,W1,H1 " + X1 + ", " + Y1 + ", " + relwidth1 + ", " + relheight1);

        Rect leftRect = new Rect((int) X1, (int) Y1, (int) relwidth1, (int) relheight1);
        Mat croppedOne = new Mat(hsvThresholdOutput, leftRect);
        leftCorrectPercent = croppedOne.total() / (double)croppedOne.rows() / (double)croppedOne.cols();
//
//        int widthTwo = boxTwoBounds[1] - boxTwoBounds[0];
//        int heightTwo = boxTwoBounds[3] - boxTwoBounds[2];

        double X2 = relBoxOne[0] * totalWidth;
        double Y2 = relBoxOne[1] * totalHeight;
        double relwidth2 = relBoxOne[2] * totalWidth;
        double relheight2 = relBoxOne[3] * totalHeight;

        Log.d(TAG, "X1,Y1,W1,H1 " + X2 + ", " + Y2 + ", " + relwidth2 + ", " + relheight2);

        Rect rightRect = new Rect((int) X1, (int) Y1, (int) relwidth1, (int) relheight1);
        Mat croppedTwo = new Mat(hsvThresholdOutput, rightRect);
        rightCorrectPercent = croppedTwo.total() / (double)croppedTwo.rows() / (double)croppedTwo.cols();

        Log.d(TAG, "Ldims" + croppedOne.dims());
        Log.d(TAG, "Rdims" + croppedOne.dims());
        Log.d(TAG, "Left total " + croppedOne.total());
        Log.d(TAG, "Right total " + croppedTwo.total());
        Log.d(TAG, "Left rows/cols " + croppedOne.rows() + "x" + croppedOne.cols());
        Log.d(TAG, "Left rows/cols " + croppedTwo.rows() + "x" + croppedTwo.cols());
        Log.d(TAG, "left correct " + leftCorrectPercent + " right correct " + rightCorrectPercent);

        leftIsBlack = leftCorrectPercent < minDetectionThreshold;
        rightIsBlack = rightCorrectPercent < minDetectionThreshold;

        // determine the position of the gold.

        // start with the odd case, if both are true choose the position with the most yellow
        if(leftIsBlack && rightIsBlack){
            //choose the position with the most yellow
            if(leftCorrectPercent > rightCorrectPercent) position = skyPositions.center;
            else if(leftCorrectPercent < rightCorrectPercent) position = skyPositions.right;
        } // otherwise look at which is / isnt true
        else if (leftIsBlack) position = skyPositions.center;
        else if (rightIsBlack) position = skyPositions.right;
        else position = skyPositions.left;


        mask(input, hsvThresholdOutput, input);
        return input;
    }
    /**
     * Softens an image using a gaussian blur
     * @param input The image on which to perform the gaussBlur.
     * @param doubleRadius The radius for the gaussBlur.
     * @param output The image in which to store the output.
     */
    private void gaussBlur(Mat input, double doubleRadius, Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        kernelSize = 6 * radius + 1;
        Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);

    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    public void init(Context context) {
        init(context, 0);
    }

    public void init(Context context, final int cameraIndex) {
        this.initStarted = true;
        this.context = context;
        final Activity activity = (Activity) context;
        final Context finalContext = context;
        final CameraBridgeViewBase.CvCameraViewListener2 self = this;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                // JCVs must be instantiated on a UI thread
                cameraView = new JavaCameraView(finalContext, cameraIndex);
                cameraView.setCameraIndex(cameraIndex);
                cameraView.setCvCameraViewListener(self);
                inited = true;
            }
        });
    }

    public void enable() {
        if (!initStarted) throw new IllegalStateException("init() needs to be called before an OpenCVPipeline can be enabled!");
        // this is an absolute hack
        try {
            while (!inited) Thread.sleep(10);
        } catch (InterruptedException e) { return; }

        cameraView.enableView();
        setCurrentView(context, cameraView);
    }

    public void disable() {
        cameraView.disableView();
        removeCurrentView(context);
    }

    public void setCurrentView(Context context, View newView) {
        final View view = cameraView;
        // finding the resID dynamically allows this class to exist outside of the TeamCode module
        final int resID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());
        final Activity activity = (Activity) context;
        final View queuedView = newView;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ViewGroup l = (ViewGroup) activity.findViewById(resID); //R.id.RelativeLayout);
                if (view != null) {
                    l.removeView(view);
                }
                l.addView(queuedView);
                cameraView = (JavaCameraView) queuedView;
            }
        });
    }

    public void removeCurrentView(Context context) {
        final View view = cameraView;
        final int resID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());
        final Activity activity = (Activity) context;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //cameraMonitorViewId
                ViewGroup l = (ViewGroup) activity.findViewById(resID); // .id.RelativeLayout);
                if (view != null) {
                    l.removeView(view);
                }
                cameraView = null;
            }
        });
    }

//    // looks at processed frame, to get location of black
//    public void findBlack(Mat input){
//        totalWidth = input.width();
//        totalHeight = input.height();
//        // find the actual size of the bounding box.
//        boxOneBounds = scaleArray(relBoxOne, totalWidth,totalHeight);
//        boxTwoBounds = scaleArray(relBoxTwo, totalWidth, totalHeight);
//        // first look in box 1.
//        // iterate throughout the box
//        int correctCounter =0; int counter =0;
//        for (int x = boxOneBounds[0];x < boxOneBounds[2]; x++){
//            for (int y = boxOneBounds[1]; y < boxOneBounds[3];y++ ){
//                counter ++;
//                if(input.get(y,x)[0] > onVal){
//                    // if it is correct, increment the correctVals Counter
//                    correctCounter ++;
//                }
//            }
//        }
//        leftCorrectPercent = ((double)correctCounter) / ((double)(counter));
//        Log.i(TAG, "leftSelection: got " +correctCounter +" hits out of " + counter + " fraction: " + leftCorrectPercent);
//
//        // reset counters
//        correctCounter = counter = 0 ;
//        // then look in box 2
//        for (int x = boxTwoBounds[0];x < boxTwoBounds[2]; x++){
//            for (int y = boxTwoBounds[1]; y < boxTwoBounds[3];y++ ) {
//                counter++;
//                if (input.get(y, x)[0] > onVal) {
//                    // if it is correct, increment the correctVals Counter
//                    correctCounter++;
//                }
//            }
//        }
//        rightCorrectPercent = ((double)correctCounter) / ((double)(counter));
//        Log.i(TAG, "rightSelection: got " +correctCounter +" hits out of " + counter + " fraction: " + rightCorrectPercent);
//
//        leftIsBlack = leftCorrectPercent > minDetectionThreshold;
//        rightIsBlack = rightCorrectPercent > minDetectionThreshold;
//
//        // determine the position of the gold.
//
//        // start with the odd case, if both are true choose the position with the most yellow
//        if(leftIsBlack && rightIsBlack){
//            //choose the position with the most yellow
//            if(leftCorrectPercent > rightCorrectPercent) position = skyPositions.center;
//            else if(leftCorrectPercent < rightCorrectPercent) position = skyPositions.right;
//        } // otherwise look at which is / isnt true
//        else if (leftIsBlack) position = skyPositions.center;
//        else if (rightIsBlack) position = skyPositions.right;
//        else position = skyPositions.left;
//    }

//    private int[]scaleArray(double[] input, int scalarX, int scalarY){
//        int[] result = new int[input.length];
//        for(int i = 0; i< input.length; i+=2){
//            result[i] = (int)(scalarX * input[i]);
//        }
//        for(int i = 1; i< input.length; i+=2){
//            result[i] = (int)(scalarY * input[i]);
//        }
//        return result;
//    }

    public skyPositions getPosition() {
        return position;
    }
}


