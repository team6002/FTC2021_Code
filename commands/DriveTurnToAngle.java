package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.util.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTurnToAngle extends CommandBase {
    private Drivetrain m_drivetrain;
    private Odometry m_odometry;
    private double m_targetX, m_targetY;
    private Rotation2d m_targetHeading=null;
    private boolean m_isFinished = false;
    private boolean m_hasTargetHeading = false;
    private double m_deltaHeading = Double.MAX_VALUE;

    private ElapsedTime runtime = new ElapsedTime();

    private double m_currentHeading;

    private double m_turnSpeed = 1;
    private double m_minTurnSpeed = .3;
    private double m_angleBuffer = Math.toRadians(2);

    // optimize these two values to achieve best balance between accuracy and speed... try using 10 degree and 100 degree turns
    private double m_turnSpeedDecelerateDegrees = 45; // how many degrees to decelerate from max voltage?
    private double m_TimeToMaxVelocity = 200;

    public DriveTurnToAngle(BaseRobot baseRobot) {
        m_drivetrain = baseRobot.m_Drivetrain;
        m_odometry = baseRobot.m_Odometry;

        m_hasTargetHeading = false;
        m_turnSpeed = 1;
        m_minTurnSpeed = .3;
        m_angleBuffer = Math.toRadians(2); // convert to radians
        m_deltaHeading = Double.MAX_VALUE;
        m_isFinished = false;

        addRequirements(m_drivetrain, m_odometry);
    }

    public DriveTurnToAngle(BaseRobot baseRobot, double targetHeading) {
        this(baseRobot);
        m_targetHeading = new Rotation2d(Math.toRadians(targetHeading)); // convert to radians
        m_hasTargetHeading = true;
    }

    public DriveTurnToAngle(BaseRobot baseRobot, double targetX, double targetY) {
        this(baseRobot);
        m_targetX = targetX;
        m_targetY = targetY;
    }

    public DriveTurnToAngle minSpeed(double p_minSpeed) {
         m_minTurnSpeed = p_minSpeed;
         return this;
    }

    public DriveTurnToAngle speed(double speed) {
         m_turnSpeed = speed;
         return this;
    }

    public DriveTurnToAngle maxSpeed(double p_maxSpeed) {
        return speed(p_maxSpeed);
    }

    public DriveTurnToAngle angleBuffer(double p_angleBuffer) {
         m_angleBuffer = Math.toRadians(p_angleBuffer);
         return this;
    }

    public DriveTurnToAngle deltaAngle(double deltaAngle) {
        m_deltaHeading = deltaAngle;
        return this;
    }

    // Old style constructors that accepted drivetrain and odometry as args instead of just "this".
    // Kept around for compatibility purposes only.
    public DriveTurnToAngle(Drivetrain drivetrain, Odometry odometry) {
        this(odometry.m_baseRobot);
    }

    public DriveTurnToAngle(Drivetrain drivetrain, Odometry odometry, double targetHeading) {
        this(odometry.m_baseRobot, targetHeading);
    }

    public DriveTurnToAngle(Drivetrain drivetrain, Odometry odometry,
                                double targetX, double targetY) {
        this(odometry.m_baseRobot, targetX, targetY);
    }

    @Override
    public void initialize() {
        if (!m_hasTargetHeading) {
            if (m_deltaHeading != Double.MAX_VALUE)
                m_targetHeading = new Rotation2d(m_odometry.getHeading() + Math.toRadians(m_deltaHeading));
            else {
                double targetAngle = Math.atan2(m_targetY - m_odometry.getY(), m_targetX - m_odometry.getX());
                m_targetHeading = new Rotation2d(targetAngle);
            }
        }

        m_isFinished = false;
    }

    @Override
    public void execute() {
        if (Math.abs(m_targetHeading.getRadians() - m_odometry.getHeading()) <= m_angleBuffer){
            m_drivetrain.stop();
            m_isFinished = true;
            return;
        }

        Rotation2d currentHeading = new Rotation2d(m_odometry.getHeading());
        Rotation2d diff = currentHeading.minus(m_targetHeading);

        double percentFullSpeed;
        double turnSpeed = m_turnSpeed; // assume max speed
        double time = runtime.milliseconds(); // how long this has been running

        // Check if we are still accelerating
        if (time  < m_TimeToMaxVelocity) {
            // if yes, calculate the percent of full speed (full voltage)
            turnSpeed = time / m_TimeToMaxVelocity;
            if (turnSpeed > m_turnSpeed) turnSpeed = m_turnSpeed;
        }

        double degreesRemaining = Math.abs(diff.getDegrees());
        if (degreesRemaining < m_turnSpeedDecelerateDegrees) {
            percentFullSpeed = degreesRemaining / m_turnSpeedDecelerateDegrees;
            turnSpeed = Math.min(turnSpeed, percentFullSpeed);
        }

        if (turnSpeed < m_minTurnSpeed) turnSpeed = m_minTurnSpeed;

        if (diff.getRadians() < 0) turnSpeed *= -1;

        // m_drivetrain.telemetry.addData("turn","Target(%.2f) diff(%.2f) turnSpeed %.2f"
        //                                     ,m_targetHeading.getDegrees(),diff.getDegrees(), turnSpeed);

        m_drivetrain.driveRobotCentric(0.0, 0.0, turnSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    @Override
    public boolean isFinished() {

        return m_isFinished;
    }
}
