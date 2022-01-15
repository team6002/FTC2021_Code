package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input.
 */
public class DriveDefault extends CommandBase {
    private final Drivetrain m_Drivetrain;
    private final Odometry m_Odometry;
    private final GamepadEx m_driverOP;
    private double m_driverOffsetAngle = 0;
    private double m_joystickMin = 0.02;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param driverOp The control input from gamepad

     */
    // public DriveDefault(Drivetrain subsystem, Odometry odometry, GamepadEx driverOp
            // , double driverOffsetAngle
            // , double p_minSpeed, double p_thresholdSpeed
            // , double p_minTurn, double p_thresholdTurn
            // , double p_minStrafe, double p_thresholdStrafe
            // , boolean p_squareInputs) {

        // m_Drivetrain = subsystem;
        // m_Odometry = odometry;
        // m_driverOP = driverOp; // gamepad of driver
        // m_driverOffsetAngle = driverOffsetAngle;
        // m_minSpeed = p_minSpeed;
        // m_thresholdSpeed = p_thresholdSpeed;
        // m_minTurn = p_minTurn;
        // m_thresholdTurn = p_thresholdTurn;
        // m_minStrafe = p_minStrafe;
        // m_thresholdStrafe = p_thresholdStrafe;
        // m_squareInputs = p_squareInputs;
        // m_slowSpeed = -1;
        // m_slowThreshold = -1;

        // addRequirements(m_Drivetrain);
    // }

    public DriveDefault(Drivetrain subsystem, Odometry odometry, GamepadEx driverOp,
                            double driverOffsetAngle, double joystickMin) {
        m_Drivetrain = subsystem;
        m_Odometry = odometry;
        m_driverOP = driverOp; // gamepad of driver
        m_driverOffsetAngle = driverOffsetAngle;
        m_joystickMin = joystickMin;

        addRequirements(m_Drivetrain);
    }

    // public DriveDefault(Drivetrain subsystem, Odometry odometry, GamepadEx driverOp
            // , double driverOffsetAngle
            // , boolean p_squareInputs) {
        // this(subsystem, odometry, driverOp, driverOffsetAngle, .18, .5, .18, .5, .18, .5, p_squareInputs);
    // }

    @Override
    public void execute() {
        double leftY = m_driverOP.getLeftY(); // speed
        double leftX = m_driverOP.getLeftX(); // strafe
        double rightX = m_driverOP.getRightX(); // turn
        double heading = m_Odometry.getDegrees() + m_driverOffsetAngle;
        double speed = 0, turn = 0, strafe = 0;

        if (Math.abs(leftY) > m_joystickMin) // mininum threshold to start moving
            speed = leftY;
        if (Math.abs(rightX) > m_joystickMin) // mininum threshold to start moving
            turn = rightX;
        if (Math.abs(leftX) > m_joystickMin) // mininum threshold to start moving
            strafe = leftX;

        final double slowMax = 0.25;
        double slowMo = m_driverOP.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        if (slowMo > slowMax) {
            speed *= slowMax / slowMo;
            strafe *= slowMax * 1.5 / slowMo;
            turn *= slowMax / slowMo;
        }

        // m_Odometry.telemetry.addData("drive: ", "leftY: %.2f, speed: %.2f, leftX: %.2f, strafe: %.2f", leftY, speed, leftX, strafe);

        // positive strafe, move to the right
        // positive speed, move forward
        // positive turn, turn torward the right

        m_Drivetrain.driveDefaultMode(strafe, speed, turn, heading, false);
    }

}

