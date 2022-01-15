package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.BaseRobot;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
// import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


public class VuforiaNav extends SubsystemBase {

    private BaseRobot m_baseRobot;
    private boolean m_displayTelemetry = true;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    // private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    // private static final boolean PHONE_IS_PORTRAIT = false  ;

    // private OpenGLMatrix robotFromCamera;
    // private VuforiaTrackables targetsUltimateGoal;
    // private VuforiaTrackable blueTowerGoalTarget;
    // private VuforiaTrackable redTowerGoalTarget;
    // private VuforiaTrackable redAllianceTarget;
    // private VuforiaTrackable blueAllianceTarget;
    // private VuforiaTrackable frontWallTarget;
    private List<VuforiaTrackable> m_allTrackables;

    private float CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is (?) Inches in front of robot-center
    private float CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is (?) Inches above ground
    private float CAMERA_LEFT_DISPLACEMENT     = 0;   // eg: Camera (?) Inches left the robot's center line

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            // "AUaWeAL/////AAABmU79gd1bBEvUhfNuPxj4vbqIgXxEUBMAbLlPKj/T+LSVKctE9TEAEfA9aAeAagpEny32DMhyMlEkyphIN0E6WaNYwYNOqGcwbW5Yv5TFvRIPCPJQDAXqP/Ver7MxUkfRuSEfW42+khjkBF765qasOh1OrG/QZKkaUE19I+Wj97fmOp4d+3haJAOZchyCRqecQVd7G4kSEb5lWwUjJ6sOjLk1qObnyOWFUuXadhTHJVB2qVW1PCY2dG6VE9L0Agsq5BDBP3VDfSI35J1yytasn/zUv87o9s9v2hk57a3T6/0hye6H6ia4Py/vtRLQESymcuxeiw9H5NrElB2e1HAIw126pWQKqQ8onZlSrk/jTgzT";
            "ASj5adj/////AAABmWSGxHF8FEw2r7YxPWyAdh2Mk/QJwJBx1lP2OPQiPQ8NW25vDURFt71gGiDn42emZWvFlQwNfqRJoBuXS7SvBtPauFJoKMJDfpafO1dgIf7ZvcYAi0uqbY2Aix0TnPjtcTi+mBs44X6bbbv2USJ2EDJVNmh9m4Fw9oQPC+bQZk4kmqg1sISbxMsMephhmWzO/8gkLH3o0hY5NQhV/ZTIyhvyWlynnfZY9t51Xn2b+uI0NpzHSWbBCx3UzSAQGQTgQ1Tq3M+ubclp4LCczAd16BMBfI56S3mMBheY4GpIre1m24TutcWrk4NXddDrQ2mnEKfu4G76kPFPHrDAJe33B+3Ta7ZCzzpOeqm6JZXU+WDT";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    // private static final float quadField = 36 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix m_lastLocation = null;
    private VectorF m_lastLocationFtclib;
    private double m_headingFtclib;
    private double m_cameraRobotAngle = 0;

    private VuforiaLocalizer m_vuforia = null;
    private VuforiaTrackables targets = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean m_targetVisible = false;
    // private float phoneXRotate    = 0;
    // private float phoneYRotate    = 0;
    // private float phoneZRotate    = 0;

    public VuforiaNav(BaseRobot baseRobot, float p_Forward, float p_Left, float p_Vertical) {
        m_baseRobot = baseRobot;
         /*
         * Retrieve the camera we are to use.
         */
        HardwareMap hardwareMap = m_baseRobot.hardwareMap;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        m_vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.m_vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        m_allTrackables = new ArrayList<VuforiaTrackable>();
        m_allTrackables.addAll(targets);

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

        
        // Next, translate the camera lens to where it is on the robot.
        CAMERA_FORWARD_DISPLACEMENT  = (p_Forward * mmPerInch);   // eg: Camera is (?) Inches in front of robot-center
        CAMERA_VERTICAL_DISPLACEMENT = (p_Vertical * mmPerInch);   // eg: Camera is (?) Inches above ground
        CAMERA_LEFT_DISPLACEMENT     = (p_Left * mmPerInch);     // eg: Camera (?) Inches left the robot's center line

        // Create a transformation matrix describing where the phone is on the robot.
        updateCameraHeading(0);
        
        //  //Set zoom of the camera
        // com.vuforia.CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
        // com.vuforia.CameraDevice.getInstance().setField("zoom", "30");
    }

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
                setCameraLocationOnRobot(webcamName, cameraLocationOnRobot);
        }
    }

    public void activate() {
        targets.activate();
    }

    public void deactivate() {
        targets.deactivate();
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

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
