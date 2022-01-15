package org.firstinspires.ftc.teamcode.ftclib.kinematics;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Twist2d;
import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.MotorEx;
// import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.ftclib.kinematics.Odometry;

public class HolonomicOdometry extends Odometry {
  
    public double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder; // distance
    private Rotation2d previousAngle;
    private double centerWheelOffset;
    private double m_ticksToInches;
    // private boolean m_inverted;
    private boolean m_resetEncoders = false;

    // the encoders
    public MotorEx m_left, m_right, m_horizontal;

    public HolonomicOdometry(MotorEx leftEncoder, MotorEx rightEncoder,MotorEx horizontalEncoder
                , double trackWidth, double centerWheelOffset, double ticksToInches) {
        this(trackWidth, centerWheelOffset);
        m_left = leftEncoder;
        m_right = rightEncoder;
        m_horizontal = horizontalEncoder;
        m_ticksToInches = ticksToInches;
        // m_inverted = inverted;
    }

    public HolonomicOdometry(Pose2d initialPose, double trackwidth, double centerWheelOffset) {
        super(initialPose, trackwidth);
        previousAngle = initialPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
        // m_inverted = false;
    }

    public HolonomicOdometry(double trackwidth, double centerWheelOffset) {
        this(new Pose2d(), trackwidth, centerWheelOffset);
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public Pose2d updatePose() {
        // update(ticksToInches(m_left),ticksToInches(m_right),ticksToInches(m_horizontal) * (m_inverted ? -1 : 1));
        // update((m_inverted ? 1 : -1) * m_left.getDistance(), (m_inverted ? 1 : -1) * m_right.getDistance(), m_horizontal.getDistance());
        return update(-m_left.getDistance(), -m_right.getDistance(), m_horizontal.getDistance());
    }
    
    // private double ticksToInches(MotorEx motor) {
    //     return -motor.getCurrentPosition()  * m_ticksToInches;
    // }

    @Override
    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
        m_resetEncoders = true;
        
        m_left.encoder.reset();
        m_right.encoder.reset();
        m_horizontal.encoder.reset();
    }

    public Pose2d update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        if (m_resetEncoders) {
            prevLeftEncoder = 0;
            prevRightEncoder = 0;
            prevHorizontalEncoder = 0;
            m_resetEncoders = false;
        }
        
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = previousAngle.plus(
                // new Rotation2d(( deltaLeftEncoder - deltaRightEncoder) / trackWidth ) // clockwise positive
                new Rotation2d(( deltaRightEncoder - deltaLeftEncoder) / trackWidth ) // counter-clockwise positive
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());
        
        double dy = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dx = deltaHorizontalEncoder - (centerWheelOffset * dw);
        
        // Twist2d twist2d = new Twist2d(dx, dy, dw); // clockwise positive
        Twist2d twist2d = new Twist2d(dy, dx, dw); // counter-clockwise positive

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);

        return robotPose;
    }
}
