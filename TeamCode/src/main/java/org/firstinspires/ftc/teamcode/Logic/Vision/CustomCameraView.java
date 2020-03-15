package org.firstinspires.ftc.teamcode.Logic.Vision;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.util.Log;

import org.opencv.BuildConfig;
import org.opencv.android.JavaCameraView;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

public class CustomCameraView extends JavaCameraView {

    public CustomCameraView(Context context, int CameraId){
        super(context, CameraId);
    }

    public CustomCameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }


}
