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

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class SkyStoneDetector implements CameraBridgeViewBase.CvCameraViewListener2 {
    static {
        boolean magic = OpenCVLoader.initDebug();
        Log.d("ftc9773skyStoneDetector", "");
    }
    int i = 0;
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
    Boolean leftIsBlack = null, centerIsBlack = null;
    int totalWidth, totalHeight;

    // needs to be tunable
    double [] relBoxOne = {1, 1, 1, 1}; //x, y, w, h
    double [] relBoxTwo = {1, 1, 1, 1};


    double minDetectionThreshold;
    double onVal = 120.0;
    double leftCorrectPercent, centerCorrectPercent;
    final static Scalar blue = new Scalar(0,0,255);
    final static Scalar red = new Scalar(255, 0,0);

    boolean returnMat;

    skyPositions position = skyPositions.unknown;

    //Generated
    private Mat frame = new Mat();
    private Mat blur0Output = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    int X1, X2;
    int Y1, Y2;
    int relwidth1, relwidth2;
    int relheight1, relheight2;
    double total1, total2;

    public SkyStoneDetector(){
        super();
        SafeJsonReader reader = new SafeJsonReader("VisionThresholds");
        minDetectionThreshold = reader.getDouble("minDetectionThreshold", 0.1);
        // get boxOne Pos
        relBoxOne[0] = reader.getDouble("BoxOneTopLeftX",0);
        relBoxOne[1] = reader.getDouble("BoxOneTopLeftY",0);
        relBoxOne[2] = reader.getDouble("BoxOneWidth", 0.5);
        relBoxOne[3] = reader.getDouble("BoxOneLength",0.5);
        // get boxTwo Pos
        relBoxTwo[0] = reader.getDouble("BoxTwoTopLeftX",0.);
        relBoxTwo[1] = reader.getDouble("BoxTwoTopLeftY",0.);
        relBoxTwo[2] = reader.getDouble("BoxTwoWidth",0.5);
        relBoxTwo[3] = reader.getDouble("BoxTwoLength",0.5);

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
        return processFrame(inputFrame.rgba());
    }

    /***
     * Called each frame. Updates position using two filters.
     */
    public Mat processFrame(Mat input) {
        //Strip out alpha channel
        i++;
        Vector<Mat> temp = new Vector<>();
        Core.split(input, temp);
        temp.remove(temp.size() - 1);
        Core.merge(temp, input);

        // Step 1 Blur:
        Mat blur0Input = input;
        double blur0Radius = 3;
        gaussBlur(blur0Input, blur0Radius, blur0Output);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blur0Output;
        double[] hsvThresholdHue = {6.0, 44.};
        double[] hsvThresholdSaturation = {149.0, 255.0};
        double[] hsvThresholdValue = {131.0, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        totalWidth = input.width();
        totalHeight = input.height();
        Log.d(TAG, "TotalWidth " + totalWidth);
        Log.d(TAG, "TotalHeight " + totalHeight);

        X1 = (int) (relBoxOne[0] * totalWidth);
        Y1 = (int) (relBoxOne[1] * totalHeight);
        relwidth1 = (int) (relBoxOne[2] * totalWidth);
        relheight1 = (int) (relBoxOne[3] * totalHeight);

        Log.d(TAG, "X1,Y1,W1,H1 " + X1 + ", " + Y1 + ", " + relwidth1 + ", " + relheight1);

        total1 = total(blur0Output, (int)X1, (int) (X1 + relwidth1),(int) Y1, (int)(Y1 + relheight1));
        leftCorrectPercent = total1 / (relheight1 * relwidth1);

        X2 = (int) (relBoxTwo[0] * totalWidth);
        Y2 = (int) (relBoxTwo[1] * totalHeight);
        relwidth2 = (int) (relBoxTwo[2] * totalWidth);
        relheight2 = (int) (relBoxTwo[3] * totalHeight);

        Log.d(TAG, "X2,Y2,W2,H2 " + X2 + ", " + Y2 + ", " + relwidth2 + ", " + relheight2);

        total2 = total(blur0Output, (int)X2, (int) (X2 + relwidth2),(int) Y2, (int)(Y2 + relheight1));
        centerCorrectPercent = total2 / (relwidth2 * relheight2);
        double magic = hsvThresholdOutput.get(X1 + 50, Y1 + 50)[0];
        double moremagic = hsvThresholdOutput.get(X2 + 158/2, Y2 + 58/2)[0];
        Log.d(TAG, "Magic " + magic);
        Log.d(TAG, "More Magic " + moremagic);

        Log.d(TAG, "Left total " + total1);
        Log.d(TAG, "Center total " + total2);
        Log.d(TAG, "left correct " + leftCorrectPercent + " center correct " + centerCorrectPercent);
        Log.d(TAG, "Channels " + hsvThresholdOutput.channels());


        leftIsBlack = leftCorrectPercent < minDetectionThreshold;
        centerIsBlack = centerCorrectPercent < minDetectionThreshold;

        // determine the position of the gold.

        // start with the odd case, if both are true choose the position with the most yellow
        if(leftIsBlack && centerIsBlack){
            //choose the position with the most yellow
            if(leftCorrectPercent < centerCorrectPercent) position = skyPositions.left;
            else if(leftCorrectPercent >= centerCorrectPercent) position = skyPositions.center;
        } // otherwise look at which is / isnt true
        else if (leftIsBlack) position = skyPositions.left;
        else if (centerIsBlack) position = skyPositions.center;
        else position = skyPositions.right;


        input = hsvThresholdOutput;

        draw_rect(input, X1, X1 + relwidth1, Y1, Y1 + relheight1, 0);
        draw_rect(input, X2, X2 + relwidth2, Y2, Y2 + relheight2, 0);

        //Imgproc.rectangle(input, new Point(0,0), new Point(300,300), new Scalar(0,255,0));

        Log.d(TAG, "-------------------------");
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
        Core.bitwise_xor(output, mask, output);
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
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    private void toHSV(Mat input, Mat out){
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(input, out, Imgproc.COLOR_HSV2RGB);
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

    public double total(Mat input, int x_min, int x_max, int y_min, int y_max){
        double total = 0;
        int x = x_min;
        int y = y_min;
        for (; x< x_max; x++){
            for (; y<y_max;y++){
                total += input.get(x,y)[0];
            }
        }
        return total;
    }

    public void draw_rect(Mat input, int x_min, int x_max, int y_min, double y_max, int color){
        Imgproc.rectangle(input, new Point(x_min,y_min), new Point(x_max,y_max), new Scalar(0,255, color));
    }

    public skyPositions getPosition() {
        return position;
    }
}


