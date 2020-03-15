package org.firstinspires.ftc.teamcode.Logic.Vision;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

public abstract class OpenCVPipeline implements CameraBridgeViewBase.CvCameraViewListener2 {
    public static final boolean DEBUG = false;

    static {
        boolean initDebug = OpenCVLoader.initDebug();
        if (DEBUG) Log.d("OpenCVPipeline", initDebug ? " successfully init" : " failed init");
    }
    protected JavaCameraView cameraView;
    protected Context context;
    final static String TAG = "OpenCVPipeline";
    int width;
    int height;
    boolean inited = false;
    boolean initStarted = true;


    //By default initialise with phone camera
    public boolean init(Context context){return init(context, 0);}

    //Initialise the 'CameraView' to listen on. Not entirely sure how CvCameraViewListener knows which camera, but it works.EDIT: APPEARS TO BE BASED ON THE CAMERA PREVIEW ON THE PHONE
    //returns whether or not it sucessfully initialised the JavaCameraView
    public boolean init(Context context, final int cameraIndex){
        initStarted = true;
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
        return inited;
    }

    //Start Camera stream on phone screen.
    public void enable(){
        if (!initStarted) throw new IllegalStateException("init() needs to be called before an OpenCVPipeline can be enabled!");
        // this is a hack.
        try {
            while (!inited) Thread.sleep(10);
        } catch (InterruptedException e) { return;}
        cameraView.enableView();
        setCurrentView(context, cameraView);
    }

    public void disable(){
        cameraView.disableView();
        removeCurrentView(context);
    }

    public void setCurrentView(Context context, View newView){
        final View view = cameraView;
        final int resID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());
        final Activity activity = (Activity) context;
        final View queuedView = newView;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                ViewGroup l = activity.findViewById(resID); //R.id.RelativeLayout);
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

    @Override
    public void onCameraViewStarted(int width, int height) {
        this.width = width;
        this.height = height;
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public abstract Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame);
}
