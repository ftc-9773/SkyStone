package org.firstinspires.ftc.teamcode.Vision;

import android.util.Log;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

public class SkyStoneDetector extends OpenCVPipeline {

    static final String TAG = "ftc9773skyStoneDetector";
    Mat blurred ;
    Mat unblurred ;
    Mat filtered ;
    Boolean leftIsBlack = null, rightIsBlack = null;
    int totalWidth, totalHeight;

    // needs to be tunable
    double [] relBoxOne = {.1818, 0.4718137255, .3880, 0.6050857843};
    double [] relBoxTwo = {0.7761437908, 0.5208333333, 0.9803921569, 0.6740196078 };

    // actual box bounds:
    int[] boxOneBounds, boxTwoBounds;

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
    private Mat blur1Output = new Mat();
    private Mat maskOutput = new Mat();
    private Size getMatInfoSize = new Size();
    private Boolean getMatInfoEmpty;
    private Number getMatInfoChannels;
    private Number getMatInfoCols;
    private Number getMatInfoRows;
    private Number getMatInfoHighValue;


    public SkyStoneDetector(){
        super();
        SafeJsonReader reader = new SafeJsonReader("VisionThresholds");
        minDetectionThreshold = reader.getDouble("minDetectionThreshold", 0.1);
        // get boxOne Pos
        relBoxOne[0] = reader.getDouble("BoxOneTopRightX",.1818);
        relBoxOne[1] = reader.getDouble("BoxOneTopRightY",0.4718137255);
        relBoxOne[2] = reader.getDouble("BoxOneBottomLeftX",.3880);
        relBoxOne[3] = reader.getDouble("BoxOneBottomLeftY",0.6050857843);
        // get boxTwo Pos
        relBoxTwo[0] = reader.getDouble("BoxTwoTopRightX",0.7761437908);
        relBoxTwo[1] = reader.getDouble("BoxTwoTopRightY",0.5208333333);
        relBoxTwo[2] = reader.getDouble("BoxTwoBottomLeftX",0.9803921569);
        relBoxTwo[3] = reader.getDouble("BoxTwoBottomLeftY",0.6740196078);

        returnMat = reader.getBoolean("returnMat", false);


    }

    /***
     * Called each frame. Updates position using two filters.
     */
    @Override
    public Mat processFrame(Mat source0, Mat source1) {
        // Step 1 Blur:
        Mat blur0Input = source0;
        double blur0Radius = 9.90990990990991;
        gaussBlur(blur0Input, blur0Radius, blur0Output);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blur0Output;
        double[] hsvThresholdHue = {6.474820143884892, 43.63636363636364};
        double[] hsvThresholdSaturation = {149.05575539568346, 255.0};
        double[] hsvThresholdValue = {130.71043165467626, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        //TODO: CORRECT RELBOX VALUES!
        int correctCounter =0; int counter =0;
        for (int x = boxOneBounds[0];x < boxOneBounds[2]; x++){
            for (int y = boxOneBounds[1]; y < boxOneBounds[3];y++ ){
                counter ++;
                if(hsvThresholdOutput.get(y,x)[0] > onVal){
                    correctCounter ++;
                }
            }
        }
        leftCorrectPercent = ((double)correctCounter) / ((double)(counter));
        Log.i(TAG, "leftSelection: got " +correctCounter +" hits out of " + counter + " fraction: " + leftCorrectPercent);

        // reset counters
        correctCounter = counter = 0 ;
        // then look in box 2
        for (int x = boxTwoBounds[0];x < boxTwoBounds[2]; x++){
            for (int y = boxTwoBounds[1]; y < boxTwoBounds[3];y++ ) {
                counter++;
                if (hsvThresholdOutput.get(y, x)[0] > onVal) {
                    // if it is correct, increment the correctVals Counter
                    correctCounter++;
                }
            }
        }
        rightCorrectPercent = ((double)correctCounter) / ((double)(counter));

        leftIsBlack = leftCorrectPercent > minDetectionThreshold;
        rightIsBlack = rightCorrectPercent > minDetectionThreshold;

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


        return hsvThresholdOutput;
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

    // looks at processed frame, to get location of black
    public void findBlack(Mat input){
        totalWidth = input.width();
        totalHeight = input.height();
        // find the actual size of the bounding box.
        boxOneBounds = scaleArray(relBoxOne, totalWidth,totalHeight);
        boxTwoBounds = scaleArray(relBoxTwo, totalWidth, totalHeight);
        // first look in box 1.
        // iterate throughout the box
        int correctCounter =0; int counter =0;
        for (int x = boxOneBounds[0];x < boxOneBounds[2]; x++){
            for (int y = boxOneBounds[1]; y < boxOneBounds[3];y++ ){
                counter ++;
                if(input.get(y,x)[0] > onVal){
                    // if it is correct, increment the correctVals Counter
                    correctCounter ++;
                }
            }
        }
        leftCorrectPercent = ((double)correctCounter) / ((double)(counter));
        Log.i(TAG, "leftSelection: got " +correctCounter +" hits out of " + counter + " fraction: " + leftCorrectPercent);

        // reset counters
        correctCounter = counter = 0 ;
        // then look in box 2
        for (int x = boxTwoBounds[0];x < boxTwoBounds[2]; x++){
            for (int y = boxTwoBounds[1]; y < boxTwoBounds[3];y++ ) {
                counter++;
                if (input.get(y, x)[0] > onVal) {
                    // if it is correct, increment the correctVals Counter
                    correctCounter++;
                }
            }
        }
        rightCorrectPercent = ((double)correctCounter) / ((double)(counter));
        Log.i(TAG, "rightSelection: got " +correctCounter +" hits out of " + counter + " fraction: " + rightCorrectPercent);

        leftIsBlack = leftCorrectPercent > minDetectionThreshold;
        rightIsBlack = rightCorrectPercent > minDetectionThreshold;

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
    }

    private int[]scaleArray(double[] input, int scalarX, int scalarY){
        int[] result = new int[input.length];
        for(int i = 0; i< input.length; i+=2){
            result[i] = (int)(scalarX * input[i]);
        }
        for(int i = 1; i< input.length; i+=2){
            result[i] = (int)(scalarY * input[i]);
        }
        return result;
    }

    public skyPositions getPosition() {
        return position;
    }
}


