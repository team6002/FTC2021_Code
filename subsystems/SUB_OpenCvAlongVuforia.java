package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class SUB_OpenCvAlongVuforia extends SubsystemBase {
    private BaseRobot m_baseRobot;
    // private OpenCvWebcam m_webcam;
    private String m_webCameraName;
    private boolean m_displayTelemetry = true;

    private float CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is (?) Inches in front of robot-center
    private float CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is (?) Inches above ground
    private float CAMERA_LEFT_DISPLACEMENT     = 0;   // eg: Camera (?) Inches left the robot's center line


    private static final String VUFORIA_KEY =
            "ASj5adj/////AAABmWSGxHF8FEw2r7YxPWyAdh2Mk/QJwJBx1lP2OPQiPQ8NW25vDURFt71gGiDn42emZWvFlQwNfqRJoBuXS7SvBtPauFJoKMJDfpafO1dgIf7ZvcYAi0uqbY2Aix0TnPjtcTi+mBs44X6bbbv2USJ2EDJVNmh9m4Fw9oQPC+bQZk4kmqg1sISbxMsMephhmWzO/8gkLH3o0hY5NQhV/ZTIyhvyWlynnfZY9t51Xn2b+uI0NpzHSWbBCx3UzSAQGQTgQ1Tq3M+ubclp4LCczAd16BMBfI56S3mMBheY4GpIre1m24TutcWrk4NXddDrQ2mnEKfu4G76kPFPHrDAJe33B+3Ta7ZCzzpOeqm6JZXU+WDT";
    
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;
    
    // OpenCvWebcam webcam;
    OpenCvCamera vuforiaPassthroughCam;


    private OpenGLMatrix m_lastLocation = null;
    private VectorF m_lastLocationFtclib;
    private double m_headingFtclib;
    private double m_cameraRobotAngle = 0;

    // Class Members
    private VuforiaLocalizer m_vuforia    = null;
    private VuforiaTrackables m_targets   = null ;
    private WebcamName m_webcamName       = null;


    private List<VuforiaTrackable> m_allTrackables;

    private boolean m_targetVisible = false;
    private Pipeline m_pipeline;

    
    private boolean m_debugMode = false; // set true to output to the cameraMonitorView, only one camera at a time

    public SUB_OpenCvAlongVuforia(BaseRobot p_baseRobot, final String p_webCameraName
            , float p_Forward, float p_Left, float p_Vertical) 
    {
        m_baseRobot = p_baseRobot;
        m_webCameraName= p_webCameraName;
        HardwareMap hardwareMap = m_baseRobot.hardwareMap;
        m_webcamName = hardwareMap.get(WebcamName.class, p_webCameraName);
        
        VuforiaLocalizer.Parameters parameters;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 4, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        /*
         * Setup Vuforia on the webcam
         */
        if (m_debugMode) {
            parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[1]);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        };

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK; //required for webcam
        // We also indicate which camera we wish to use.
        parameters.cameraName = m_webcamName;
        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        // parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        m_vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        
        m_targets = m_vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        m_allTrackables = new ArrayList<VuforiaTrackable>();
        m_allTrackables.addAll(m_targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

/*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        // Next, translate the camera lens to where it is on the robot.
        CAMERA_FORWARD_DISPLACEMENT  = (p_Forward * mmPerInch);   // eg: Camera is (?) Inches in front of robot-center
        CAMERA_VERTICAL_DISPLACEMENT = (p_Vertical * mmPerInch);   // eg: Camera is (?) Inches above ground
        CAMERA_LEFT_DISPLACEMENT     = (p_Left * mmPerInch);     // eg: Camera (?) Inches left the robot's center line

        // Create a transformation matrix describing where the phone is on the robot.
        updateCameraHeading(0);

        float phoneXRotate = 90;
        float phoneYRotate = (float) (m_cameraRobotAngle - 90);
        float phoneZRotate = 0;

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT,
                             CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX,
                            DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : m_allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        m_targets.activate();
        // Create a Vuforia passthrough "virtual camera"

        if (m_debugMode) {
            vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(m_vuforia, parameters, viewportContainerIds[0]);
        } else {
            vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(m_vuforia, parameters);    
        }            

        m_pipeline = new Pipeline();
        // webcam.setPipeline(pipeline);
        // webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        
        
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        // phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        // phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        
        vuforiaPassthroughCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                vuforiaPassthroughCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                // vuforiaPassthroughCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                vuforiaPassthroughCam.setPipeline(m_pipeline);

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                vuforiaPassthroughCam.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    };
    
    /**
     * We also need to tell Vuforia where the <em>cameras</em> are relative to the robot.
     *
     * Just as there is a Field Coordinate System, so too there is a Robot Coordinate System.
     * The two share many similarities. The origin of the Robot Coordinate System is wherever
     * you choose to make it on the robot, but typically you'd choose somewhere in the middle
     * of the robot. From that origin, the Y axis is horizontal and positive out towards the
     * "front" of the robot (however you choose "front" to be defined), the X axis is horizontal
     * and positive out towards the "right" of the robot (i.e.: 90deg horizontally clockwise from
     * the positive Y axis), and the Z axis is vertical towards the sky.
     *
     * Similarly, for each camera there is a Camera Coordinate System. The origin of a Camera
     * Coordinate System lies in the middle of the sensor inside of the camera. The Z axis is
     * positive coming out of the lens of the camera in a direction perpendicular to the plane
     * of the sensor. When looking at the face of the lens of the camera (down the positive Z
     * axis), the X axis is positive off to the right in the plane of the sensor, and the Y axis
     * is positive out the top of the lens in the plane of the sensor at 90 horizontally
     * counter clockwise from the X axis.
     *
     * Next, there is Phone Coordinate System (for robots that have phones, of course), though
     * with the advent of Vuforia support for Webcams, this coordinate system is less significant
     * than it was previously. The Phone Coordinate System is defined thusly: with the phone in
     * flat front of you in portrait mode (i.e. as it is when running the robot controller app)
     * and you are staring straight at the face of the phone,
     *     * X is positive heading off to your right,
     *     * Y is positive heading up through the top edge of the phone, and
     *     * Z is pointing out of the screen, toward you.
     * The origin of the Phone Coordinate System is at the origin of the Camera Coordinate System
     * of the front-facing camera on the phone.
     *
     * Finally, it is worth noting that trackable Vuforia Image Targets have their <em>own</em>
     * coordinate system (see {@link VuforiaTrackable}. This is sometimes referred to as the
     * Target Coordinate System. In keeping with the above, when looking at the target in its
     * natural orientation, in the Target Coodinate System
     *     * X is positive heading off to your right,
     *     * Y is positive heading up through the top edge of the target, and
     *     * Z is pointing out of the target, toward you.
     *
     * One can observe that the Camera Coordinate System of the front-facing camera on a phone
     * coincides with the Phone Coordinate System. Further, when a phone is placed on its back
     * at the origin of the Robot Coordinate System and aligned appropriately, those coordinate
     * systems also coincide with the Robot Coordinate System. Got it?
     *
     * In this example here, we're going to assume that we put the camera on the right side
     * of the robot (facing outwards, of course). To determine the transformation matrix that
     * describes that location, first consider the camera as lying on its back at the origin
     * of the Robot Coordinate System such that the Camera Coordinate System and Robot Coordinate
     * System coincide. Then the transformation we need is
     *      * first a rotation of the camera by +90deg along the robot X axis,
     *      * then a rotation of the camera by +90deg along the robot Z axis, and
     *      * finally a translation of the camera to the side of the robot.
     *
     * When determining whether a rotation is positive or negative, consider yourself as looking
     * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
     * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
     * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
     * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
     */

    // Note: Vuforia coordinate system on an FTC field has 90 pointing to the red alliance side, 180 pointing to
    // the front (non-audience) side, 0 pointing to the back, and -90 pointing to the blue alliance side.
    // cameraRobotAngle is the direction the camera is pointing relative to the robot (using FTC coordinates).
    // 0 means it is pointing forward.  90 means it is pointing to the left side of the robot.
    // -90 means right side.  If your camera can rotate, call this method passing in the new cameraRobotAngle
    // whenever your camera's heading changes.  Then have Vuforia re-scan for targets.  The returned FTC pose
    // takes the new camera angle into account.
    public void updateCameraHeading(double cameraRobotAngle) {

        // Camera starts with the Logitech lens facing straight up with the top of
        // camera pointing to the robot's right.  Using a phoneXRotate of 90 swings the
        // camera CCW so the lens points to the left side of the robot.  Using a phoneYRotate
        // of -90 swings the camera CW so the lens points to the front of the robot.
        float phoneXRotate = 90;
        float phoneYRotate = (float) (cameraRobotAngle - 90);
        float phoneZRotate = 0;

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT,
                             CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX,
                            DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : m_allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).
                setCameraLocationOnRobot(m_webcamName, cameraLocationOnRobot);
        }
    }

    public void activate() {
        m_targets.activate();
    }

    public void deactivate() {
        m_targets.deactivate();
    }

    public boolean seeTarget() {
        return m_targetVisible;
    }

    public VectorF getRobotLocation() {
        return m_lastLocation.getTranslation();
    }


    public VectorF convertRobotLocationFtclib(VectorF p_robotLocation) {
         VectorF vuforiaLocation = p_robotLocation;
         float xFtclib = vuforiaLocation.get(0) + (72 * mmPerInch); // x = x +72in
         float yFtclib = vuforiaLocation.get(1); // y = -y
         float zFtclib = vuforiaLocation.get(2);

         return new VectorF(xFtclib / mmPerInch, yFtclib/ mmPerInch, zFtclib/ mmPerInch);
    }

    // Normalizes any number to an arbitrary range by assuming the range
    // wraps around when going below min or above max.
    private double normalize(double value, double start, double end ) {
        final double width       = end - start   ;   // 
        final double offsetValue = value - start ;   // value relative to 0

        if (width == 0)
            return 0;

        // + start to reset back to start of original range
        return (offsetValue - (Math.floor(offsetValue / width) * width)) + start;
    }

    public double convertOrientationFtclib(double heading) {
        // if (heading > 0) heading = -(180-heading);
        // else heading = (180+heading);
        // return heading;

        // Convert the Vuforia heading into an FTC heading taking into account
        // the direction the camera lens is facing. Normalize this between
        // -180 and 180.
        return normalize(heading - (m_cameraRobotAngle - 180), -180, 180);
    }

    public VectorF getRobotLocationFtclib() {
         return m_lastLocationFtclib;
    }

    public double getHeadingFtclib() {
        return m_headingFtclib;
    }

    public void periodic()  {
        // scan all trackable, only needed for troubleshooting/calibration.
        // Should be turned off once verything is running correctly
        scanAllTrackables();
    }

    public void scanAllTrackables()  {

        // check all the trackable targets to see which one (if any) is visible.
        m_targetVisible = false;
        for (VuforiaTrackable trackable : m_allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                if (m_displayTelemetry) m_baseRobot.telemetry.addData("Visible Target", trackable.getName());
                m_targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    m_lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (m_targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = m_lastLocation.getTranslation();
            m_lastLocationFtclib = convertRobotLocationFtclib(translation);

            Orientation rotation = Orientation.getOrientation(m_lastLocation, EXTRINSIC, XYZ, DEGREES);
            m_headingFtclib = convertOrientationFtclib(rotation.thirdAngle);

            if (m_displayTelemetry) {
                m_baseRobot.telemetry.addData("Ftc", "X(%.1f)  Y(%.1f)  Heading(%.1f)",
                    m_lastLocationFtclib.get(0), m_lastLocationFtclib.get(1), m_headingFtclib);
                m_baseRobot.telemetry.addData("Vuf", "X(%.1f)  Y(%.1f)  Heading(%.1f)",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle);
            }

            // m_baseRobot.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", m_rotationFtclib.firstAngle, m_rotationFtclib.secondAngle, m_rotationFtclib.thirdAngle);
            // express the rotation of the robot in degrees.
            // m_baseRobot.telemetry.addData("RotV(deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            if (m_displayTelemetry)
                m_baseRobot.telemetry.addData("Visible Target", "none");
        }
    }    

    public static class Pipeline extends OpenCvPipeline
    {

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(10,320);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(300,320);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(590,320);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile int position = 3;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = 1; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = 2; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = 3; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public int getAnalysis()
        {
            return position;
        }
    }
    
    public int getAnalysis()
    {
        return m_pipeline.getAnalysis();
    }
    
    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = m_targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }    

}