package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.ftclib.purepursuit.PathMotionProfile;
import org.firstinspires.ftc.teamcode.ftclib.purepursuit.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.util.*;

public class CMD_DrivePositionIntake extends CommandBase {

    private Drivetrain m_drivetrain;
    private Odometry m_odometry;
    private BaseRobot m_baseRobot;

    private ElapsedTime m_runtime = new ElapsedTime();

    private double m_targetX;
    private double m_targetY;
    private Translation2d m_targetTranslation;
    private double m_tolerant;
    private boolean m_isFinished = false;
    private double m_totalDistance;
    
    private double m_timeout;

    private double m_currentHeading;
    private RateLimiter m_accelerateLimiter = new RateLimiter();

    SUB_Sensor_IRBucket m_Sensor_IRBucket;

    private double m_maxSpeed;
    private double m_turnSpeed;
    private double m_minSpeed;

    private double m_positionBufferX;
    private double m_positionBufferY;
    private double m_rotationBuffer;

    // True if this uses a preferred angle.
    private boolean m_usePreferredAngle;
    private double m_preferredAngle;

    // Motion profile.
//    private PathMotionProfile m_motionProfile;
    private double m_distanceToDecelerate = 20; // inches to zero from max speed
    private double m_TimeToMaxAcceleration = 350; // the time it takes to reach full volt speed
//    private double m_distanceToAccelerate = 3; // inches to max speed

    // Provide X,Y only. Drives at full speed and spins to face the target
    // location.  Uses normal deceleration.
    public CMD_DrivePositionIntake(BaseRobot baseRobot
            , SUB_Sensor_IRBucket p_Sensor_IRBucket
            , double targetX, double targetY) {
        m_baseRobot = baseRobot;
        m_drivetrain = m_baseRobot.m_Drivetrain;
        m_odometry = m_baseRobot.m_Odometry;
        m_Sensor_IRBucket= p_Sensor_IRBucket;;

        m_maxSpeed = 1.0;
        m_turnSpeed = 1.0;
        m_minSpeed = 0.3;
        m_positionBufferX = 2.5;
        m_positionBufferY = 2.5;
        m_rotationBuffer = 5.0;
        m_distanceToDecelerate = 20;
        m_preferredAngle = 0;
        m_usePreferredAngle = false;

        m_targetX = targetX;
        m_targetY = targetY;

        addRequirements(m_drivetrain);
    }

    public CMD_DrivePositionIntake(Drivetrain drivetrain, Odometry odometry
                        , SUB_Sensor_IRBucket p_Sensor_IRBucket
                        , double targetX, double targetY) {
        this(odometry.m_baseRobot, p_Sensor_IRBucket, targetX, targetY);
    }

    public CMD_DrivePositionIntake decel(double distance) {
        m_distanceToDecelerate = distance;
        return this;
    }

    public CMD_DrivePositionIntake timeout(double timeout) {
        m_timeout = timeout;
        return this;
    }

    // Same as above but with a longer, more descriptive name.
    public CMD_DrivePositionIntake decelDistance(double distance) {
        return decel(distance);
    }

    public CMD_DrivePositionIntake angle(double angle) {
        m_preferredAngle = Math.toRadians(angle);
        m_usePreferredAngle = true;
        return this;
    }

    public CMD_DrivePositionIntake posBuffer(double buffer) {
        m_positionBufferX = buffer;
        m_positionBufferY = buffer;
        return this;
    }

    public CMD_DrivePositionIntake posBufferX(double buffer) {
        m_positionBufferX = buffer;
        return this;
    }

    public CMD_DrivePositionIntake posBufferY(double buffer) {
        m_positionBufferY = buffer;
        return this;
    }

    public CMD_DrivePositionIntake angleBuffer(double buffer) {
        m_rotationBuffer = buffer;
        return this;
    }

    public CMD_DrivePositionIntake speed(double speed) {
        m_maxSpeed = speed;
        return this;
    }

    public CMD_DrivePositionIntake minSpeed(double speed) {
        m_minSpeed = speed;
        return this;
    }

    public CMD_DrivePositionIntake turnSpeed(double speed) {
        m_turnSpeed = speed;
        return this;
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_targetTranslation = new Translation2d(m_targetX, m_targetY);
        m_totalDistance = Math.hypot(m_odometry.getY() - m_targetY, m_odometry.getX() - m_targetX);
        m_runtime.reset();
        // m_motionProfile = getDefaultMotionProfile();
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        if (m_Sensor_IRBucket.getState())
        {
            m_isFinished = true;
            return;
        }

        
        if ( (m_timeout > 0) && (m_timeout < m_runtime.milliseconds()) ) 
        {
            m_drivetrain.stop();
            m_isFinished = true;
            return;
        };

        


        // if (PurePursuitUtil.positionEqualsWithBuffer(m_odometry.getTranslation(),m_targetTranslation,m_positionBuffer)){
        if (positionEqualsWithBufferXY(m_odometry.getTranslation(),m_targetTranslation,m_positionBufferX, m_positionBufferY))
        {
            m_drivetrain.stop();
            m_isFinished = true;
            return;
        }

        double cx = m_odometry.getX();
        double cy = m_odometry.getY();
        double ca = m_odometry.getHeading();  // need clockwise positive

        double tx = m_targetX;
        double ty = m_targetY;
        double ta; // target angle

        if (m_usePreferredAngle) ta = m_preferredAngle;
        else ta = Math.atan2(ty - cy, tx - cx); // Calculate the target angle.

        double[] motorPowers = PurePursuitUtil.moveToPosition(cx, cy, ca, tx, ty, ta, false);

        double distanceLeft = Math.hypot(ty - cy, tx - cx);

//        m_baseRobot.telemetry.addData("powers raw", "(%.2f, %.2f, %.2f) max: %f min: %f left: %.2f",
//                                     motorPowers[0], motorPowers[1], motorPowers[2], m_maxSpeed, m_minSpeed, distanceLeft);
        capMotorSpeeds(motorPowers, m_maxSpeed, m_runtime.milliseconds(), distanceLeft);

        m_drivetrain.driveRobotCentric(motorPowers[0], motorPowers[1], motorPowers[2]);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    /**
     * Calculates whether or not two points are equal within a margin of error.
     *
     * @param p1 Point 1
     * @param p2 Point 2
     * @param bufferX Margin of error in X direction.
     * @param bufferY Margin of error in Y direction.
     * @return True if the point are equal within a margin or error, false otherwise.
     */
    public static boolean positionEqualsWithBufferXY(Translation2d p1, Translation2d p2, double bufferX, double bufferY) {
        if (p1.getX() - bufferX < p2.getX() && p1.getX() + bufferX > p2.getX())
            if (p1.getY() - bufferY < p2.getY() && p1.getY() + bufferY > p2.getY())
                return true;
        return false;
    }

    public void capMotorSpeeds(double[] speeds, double maxSpeed, double time, double distanceLeft) {
        double percentFullSpeed = -1;
        double currentSpeed = maxSpeed;

        // Check if we are still accelerating
        if (time < m_TimeToMaxAcceleration) {
            // if yes, calculate the percent of full speed (full voltage)
            percentFullSpeed = (time / m_TimeToMaxAcceleration);

            currentSpeed = Math.min(currentSpeed, percentFullSpeed);
        }

        if (distanceLeft < m_distanceToDecelerate) {
            percentFullSpeed = (distanceLeft / m_distanceToDecelerate);
            currentSpeed = Math.min(currentSpeed, percentFullSpeed);
        }

        if (currentSpeed < m_minSpeed) currentSpeed = m_minSpeed;

//        m_baseRobot.telemetry.addData("capped", "time: %.2f percent: %.2f speed: %.2f",
//                                     time, percentFullSpeed, currentSpeed);
        // Normalizes the speeds
        normalizeMotorSpeeds(speeds, currentSpeed, m_turnSpeed);

//        m_baseRobot.telemetry.addData("normalized", "(%.2f, %.2f, %.2f)",
//                                     speeds[0], speeds[1], speeds[2]);
    }

    /**
     * Normalizes the provided motor speeds to be in the range [-1, 1].
     * @param speeds Motor speeds to normalize.
     */
    private void normalizeMotorSpeeds(double[] speeds, double maxSpeed, double turnSpeed) {
        if (speeds[0]!=0 && speeds[1]!=0) {
            // get the abs max value
            double max = Math.max(Math.abs(speeds[0]), Math.abs(speeds[1]));
            // normalize just the strafe and forward speed
            if (max > 1) {
                speeds[0] /= max;
                speeds[1] /= max;
            }

            if (maxSpeed < m_minSpeed)
                maxSpeed = m_minSpeed;

            speeds[0] *= maxSpeed;
            speeds[1] *= maxSpeed;

            // Say forward speed was calculated to be 0.2, strafe was 0.1, and minSpeed is 0.3.  Instead of
            // just setting both speeds to .3, keep the ratio the same.  Bring forward speed up to 0.3 by multiplying
            // forward speed by 0.3/0.2 which is 0.3.  Multiply strafe by the same ratio: 0.3/0.2*0.1 = 0.15.
            if (speeds[0] != 0 && Math.abs(speeds[0]) < m_minSpeed && Math.abs(speeds[0]) > Math.abs(speeds[1])) {
                double scale = m_minSpeed / Math.abs(speeds[0]);
                speeds[0] *= scale;
                speeds[1] *= scale;
            }
            else if (speeds[1] != 0 && Math.abs(speeds[1]) < m_minSpeed && Math.abs(speeds[1]) > Math.abs(speeds[0])) {
                double scale = m_minSpeed / Math.abs(speeds[1]);
                speeds[0] *= scale;
                speeds[1] *= scale;
            }
        };

        // Clip the turnSpeed to -1, 1.  turnSpeed isn't tied to forward or strafe speed.
        if (speeds[2] > 1)
            speeds[2] = 1;
        else if (speeds[2] < -1)
            speeds[2] = -1;

        // turnSpeed needs to be at least minSpeed but the calculation is done separate from forward and strafe speed.
        if (turnSpeed < m_minSpeed)
            turnSpeed = m_minSpeed;

        speeds[2] *= turnSpeed;
    }

    /**
     * Normalizes the provided motor speeds to be in the range [-1, 1].
     * @param speeds Motor speeds to normalize.
     */
    public static void normalizeMotorSpeeds(double[] speeds) {
        double max = Math.abs(speeds[0]);

        for(int i=1; i< speeds.length;i++) {
            double temp = Math.abs(speeds[i]);
            if (temp > max) max = temp;
        }
        if(max > 1) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] = (speeds[i] / max);
            }
        }
    }

    /**
     * Generates and returns the default PathMotionProfile.
     * @return the default PathMotionProfile.
     */
//    private PathMotionProfile getDefaultMotionProfile() {
//
//        // Use the default motion profile. This may be updated in later versions.
//        // The default profile is a messy trapezoid(ish) curve.
//        return new PathMotionProfile() {
//            @Override
//            public void decelerate(double[] motorSpeeds, double distanceToTarget, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
//
//                if (distanceToTarget < m_distanceToDecelerate) {
//                    double scaledSpeed = (configuredMovementSpeed * (distanceToTarget / m_distanceToDecelerate)); // + m_minSpeed;
//                    double scaledTurnSpeed = (configuredTurnSpeed * (distanceToTarget / m_distanceToDecelerate)); // + m_minSpeed;
//                    // In the last half of the decelerate distance, don't turn anymore.
//                    normalizeMotorSpeeds(motorSpeeds, scaledSpeed, scaledTurnSpeed);//configuredTurnSpeed);
//                    // (distanceToTarget < m_distanceToDecelerate / 4) ? 0 : scaledTurnSpeed);//configuredTurnSpeed);
//
//                } else {
//                    normalizeMotorSpeeds(motorSpeeds,configuredMovementSpeed,configuredTurnSpeed);
//                }
//            }
//            @Override
//            public void accelerate(double[] motorSpeeds, double distanceFromSource, double speed, double configuredMovementSpeed, double configuredTurnSpeed) {
//
//                if (distanceFromSource < m_distanceToAccelerate) {
//                    double scaledSpeed = (configuredMovementSpeed * (distanceFromSource / m_distanceToAccelerate)); // + m_minSpeed;
//                    double scaledTurnSpeed = (configuredTurnSpeed * (distanceFromSource / m_distanceToAccelerate)); // + m_minSpeed;
//                    normalizeMotorSpeeds(motorSpeeds,scaledSpeed,scaledTurnSpeed);//configuredTurnSpeed);
//                } else {
//                    normalizeMotorSpeeds(motorSpeeds,configuredMovementSpeed,configuredTurnSpeed);
//                }
//            }
//        };
//    }

}


