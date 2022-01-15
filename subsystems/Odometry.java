package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.BaseRobot;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.*;

import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.ftclib.util.Direction;

public class Odometry extends SubsystemBase {

    public Pose2d m_lastVufPose = new Pose2d(-22, -23, new Rotation2d(Math.toRadians(-24)));;

    protected HolonomicOdometry m_odometry;
    public BaseRobot m_baseRobot;

    // define our constants
    private double m_TrackWidth;
    private double m_WheelDiameter;    // inches
    private double m_CenterWheelOffset;
    private double m_TickPerInches;

    private final MotorEx m_leftEncoder;
    private final MotorEx m_rightEncoder;
    private final MotorEx m_centerEncoder;

    /**
     * Make sure you are using the supplier version of the constructor
     *
     * @param odometry the odometry on the robot
     */
    public Odometry(BaseRobot baseRobot, final String leftEncoderName, String rightEncoderName, String centerEncoderName
            ,double trackWidth, double wheelDiameter, double centerWheelOffset, double ticksPerRotation) {

        m_baseRobot = baseRobot;

        m_leftEncoder = new MotorEx(m_baseRobot.hardwareMap, leftEncoderName);
        m_rightEncoder = new MotorEx(m_baseRobot.hardwareMap, rightEncoderName);
        m_centerEncoder = new MotorEx(m_baseRobot.hardwareMap, centerEncoderName);

        m_TrackWidth = trackWidth;
        m_WheelDiameter = wheelDiameter;
        m_CenterWheelOffset = centerWheelOffset;
        m_TickPerInches = wheelDiameter * Math.PI / ticksPerRotation;

        // m_leftEncoder.encoder.setDirection(Motor.Direction.FORWARD);
        // m_rightEncoder.encoder.setDirection(Motor.Direction.FORWARD);
        // m_centerEncoder.encoder.setDirection(Motor.Direction.FORWARD);

        m_leftEncoder.setDistancePerPulse(m_TickPerInches);
        m_rightEncoder.setDistancePerPulse(m_TickPerInches);
        m_centerEncoder.setDistancePerPulse(m_TickPerInches);

        m_leftEncoder.encoder.reset();
        m_rightEncoder.encoder.reset();
        m_centerEncoder.encoder.reset();

        m_odometry = new HolonomicOdometry(m_leftEncoder, m_rightEncoder,
                        m_centerEncoder, trackWidth, centerWheelOffset, m_TickPerInches);
    }

    public void invertEncoders(boolean invertLeft, boolean invertRight, boolean invertCenter) {
        if (invertLeft)
            m_leftEncoder.encoder.setDirection(Motor.Direction.REVERSE);
        if (invertRight)
            m_rightEncoder.encoder.setDirection(Motor.Direction.REVERSE);
        if (invertCenter)
            m_centerEncoder.encoder.setDirection(Motor.Direction.REVERSE);
    }

    public Pose2d getPose() {
        return m_odometry.getPose();
    }

    public Translation2d getTranslation(){
        return (m_odometry.getPose().getTranslation());
    }

    public double getX() {
        return m_odometry.getPose().getX();
    }

    public double getY() {
        return m_odometry.getPose().getY();
    }

    public double getHeading() {
        return m_odometry.getPose().getHeading();  // in radians
    }

    public double getDegrees() {
        return Math.toDegrees(getHeading());  // in degrees
    }

    public void updatePose(double x, double y, double rotation) {  // rotation in radians
        Pose2d pose = new Pose2d(x,y, new Rotation2d(rotation));
        m_odometry.updatePose(pose);
    }

    public void updatePose(Pose2d pose) {  // rotation in radians
        m_odometry.updatePose(pose);
    }


    /**
     * Call this at the end of every loop
     */
    public void updatePose() {
        m_odometry.updatePose();
    }


    public void telemetry() {

        m_baseRobot.telemetry.addData("Odometry", "(%.2f, %.2f, %.2f)",
                                   getX(), getY(), Math.toDegrees(getHeading()));
        // m_baseRobot.telemetry.addData("Odometry", "x: %.2f, y: %.2f, h: %.2f",
        //                              getX(), getY(), Math.toDegrees(getHeading()));
        // m_baseRobot.telemetry.addData("Odometry", "XY (%.2f,%.2f) Heading(%.2f) last pose: (%.1f, %.1f, %.1f)"
        //                         ,getX(),getY(),Math.toDegrees(getHeading()), m_lastVufPose.getX(), m_lastVufPose.getY(), Math.toDegrees(m_lastVufPose.getHeading()));

        m_baseRobot.telemetry.addData("encoders: ", "L/R/H (%.1f, %.1f, %.1f)",
                        m_odometry.prevLeftEncoder, m_odometry.prevRightEncoder, m_odometry.prevHorizontalEncoder);

        m_baseRobot.telemetry.addData("ticks", String.format("left: %d, right: %d, horz: %d\n",
                                  m_odometry.m_left.getCurrentPosition()
                                  , m_odometry.m_right.getCurrentPosition()
                                  , m_odometry.m_horizontal.getCurrentPosition()));
    }


    /**
     * Updates the pose every cycle
     */

    @Override
    public void periodic() {
        // After every update, store the last known pose in a global so
        // it can be retrieved between OpModes.
        m_baseRobot.m_globalVariables.setRobotPose(m_odometry.updatePose());
        telemetry();
    }

    public void resetEncoder() {
        m_leftEncoder.resetEncoder();
        m_rightEncoder.resetEncoder();
        m_centerEncoder.resetEncoder();
    }

    public OpMode opMode() {
        return m_baseRobot;
    }
}
