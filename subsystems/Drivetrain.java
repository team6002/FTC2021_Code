package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.BaseRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.Motor.Encoder;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.Motor;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.Motor.ZeroPowerBehavior;


public class Drivetrain extends SubsystemBase {

    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private BaseRobot m_baseRobot;
    private boolean m_invertStrafeDirection = false;
    private boolean m_invertHeading = false;
    // private boolean m_inverted = false;
    public boolean m_modeFieldCentric = true;
    private int m_controlBackward = 1; //  1= normal control, -1 = backware control
    
    /**
     * Creates a new DriveSubsystem.
     */
    public Drivetrain(MotorEx LeftFront,MotorEx RightFront, MotorEx LeftBack, MotorEx RightBack) {
        fL = LeftFront;
        fR = RightFront;
        bL = LeftBack;
        bR = RightBack;

        fL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        m_robotDrive = new MecanumDrive(LeftFront, RightFront, LeftBack, RightBack);
    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public Drivetrain(BaseRobot baseRobot, final String leftFrontMotorName, String rightFrontMotorName,
                          String leftBackMotorName, String rightBackMotorName) {

        this(new MotorEx(baseRobot.hardwareMap, leftFrontMotorName), new MotorEx(baseRobot.hardwareMap, rightFrontMotorName),
                    new MotorEx(baseRobot.hardwareMap,leftBackMotorName), new MotorEx(baseRobot.hardwareMap, rightBackMotorName));

        m_baseRobot = baseRobot;
    }

    public void invertDriveMotors(boolean invertLeftFront, boolean invertRightFront, boolean invertLeftBack, boolean invertRightBack) {
        fL.setInverted(invertLeftFront);
        fR.setInverted(invertRightFront);
        bL.setInverted(invertLeftBack);
        bR.setInverted(invertRightBack);
    }

    
    public void setControlNormal() {
        m_controlBackward=1;
    }
    
    public void setControlBackward() {
        m_controlBackward=-1;
    }

    public void setControlToggle() {
        m_controlBackward*=-1;
    }


    public void invertStrafeDirection(boolean invert) {
        m_invertStrafeDirection = invert;
    }

    public void invertHeading(boolean invert) {
        m_invertHeading = invert;
    }

    // public void invertRobotOrientation() {
        // fL.setInverted(true);
        // fR.setInverted(true);
        // bL.setInverted(true);
        // bR.setInverted(true);
        // m_inverted = true;
    // }

    public void driveDefaultMode(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, boolean squareInputs) {
        if (m_modeFieldCentric)
            driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading, squareInputs);
        else
            driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, squareInputs);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        m_robotDrive.driveRobotCentric(m_controlBackward * (m_invertStrafeDirection ? -1 : 1) * strafeSpeed, m_controlBackward * forwardSpeed, turnSpeed);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        m_robotDrive.driveRobotCentric(m_controlBackward * (m_invertStrafeDirection ? -1 : 1) * strafeSpeed, m_controlBackward * forwardSpeed, turnSpeed, squareInputs);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, boolean squareInputs) {
        m_robotDrive.driveFieldCentric((m_invertStrafeDirection ? -1 : 1) * strafeSpeed, forwardSpeed, turnSpeed,
                                      (m_invertHeading ? -1 : 1) * heading, squareInputs);

        // m_robotDrive.driveFieldCentric((m_invertStrafeDirection ? -1 : 1) * strafeSpeed, forwardSpeed, turnSpeed,
        // m_robotDrive.driveFieldCentric(-strafeSpeed, forwardSpeed, turnSpeed,
        //                               (m_invertStrafeDirection ? 1 : -1) * heading, squareInputs);
    }

    public void switchDriveMode(boolean fieldCentric) {
        m_modeFieldCentric = fieldCentric;
    }

    public void stop() {
        m_robotDrive.stop();
    }

    public void telemetry() {
        // m_baseRobot.telemetry.addData("Drive:","Drive");

    }
    @Override
    public void periodic() {

    }
}
