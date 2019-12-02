package org.firstinspires.ftc.teamcode.Logic.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * For detecting the skystone.
 * Most of this is copied from "FtcRobotController.org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaSkyStoneNavigation"
 * */
@Deprecated
//TODO: DOESN"T ACTUALLY WORK! USE OPENCV for skystone. MAYBE FOR PRECISE ROBOT LOCALISATION
public class VuforiaDetector {
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    //Ask Cadence if the key goes missing. URL: https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY =
            "ARtebAH/////AAABmWjfZr9vKk9ToTt+qDl7jwtl7iYXofviYrjP5U2YMRCos5y4LfFJvnT1VnU2yWyQJ3W4ipfAXNr9Xi4eqTvHmIXsSCMphdWANlKbrwWCop5yYfRUkc9qm6d6xodVUXu83C9/4GE2og8N37YofgBqROPOf52lGcRfoiZFUDs+boK272IfhsIJW4msW+TcSSr+kMf5n1ZnvBOy038TT1vtlzLAZEdl2LxkuuWhJ1TjpRoDf+69xx5TF6grrwl+FB1rhxg3w5YRmeVKxLnvefncRzcUQ48zzjZaWslTM1ohAsiww/MyoLpG/OrFqXy2biPMa/kRZYsl2JNzWyLXHe818Arz0yduaZ0dQOMiThs3tpgw";
    private static final float mm_PER_INCH = 25.4f;
    private static final float mm_TARGET_HEIGHT = (6) * mm_PER_INCH;          // the height of the center of the target image above the floor
    private static final float skyStoneZ = 2.00f * mm_PER_INCH;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    //TODO GET THESE VALUES FROM JSON FILE
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VuforiaTrackable target;

    enum POSITION {ONE, TWO, THREE, UNKNOWN}

    public VuforiaDetector(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        target = stoneTarget;

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, skyStoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mm_PER_INCH;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mm_PER_INCH;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        //Convert rel2camera to coordi
        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
    }

    public POSITION detect(){
        if (((VuforiaTrackableDefaultListener) target.getListener()).isVisible()){
            POSITION position = POSITION.UNKNOWN;
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)target.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
            VectorF translation = lastLocation.getTranslation();

            return position;
        } else {
            return POSITION.UNKNOWN;
        }
    }
}
