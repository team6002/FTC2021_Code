package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.DoubleSupplier;
import java.util.ArrayList;
import java.util.List;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes.
 */
public class OdometryUpdByVuforia extends CommandBase {

    private ElapsedTime m_runtime = new ElapsedTime();
    private final Odometry m_Odometry;
    private final VuforiaNav m_VuforiaNav;
    private LED m_LED;
    private double m_Maxtime = 1000;
    private boolean m_shouldBeClose = true;
    private int m_numSimilar;
    private RevBlinkinLedDriver.BlinkinPattern m_ledColor;

    List<Pose2d> m_poseArray = new ArrayList<Pose2d>();

    public OdometryUpdByVuforia(Odometry p_odometry, VuforiaNav p_VuforiaNav) {
        m_Odometry = p_odometry;
        m_VuforiaNav = p_VuforiaNav;
        m_Maxtime = 1000;
        m_shouldBeClose = false;
        m_numSimilar = 3;
        m_ledColor = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        addRequirements(m_Odometry);
    }

    // If you don't have the LED subsystem, comment out this method plus the other two lines
    // that reference m_LED.
    public OdometryUpdByVuforia led(LED led) {
        m_LED = led;
        return this;
    }

    // Override the default green LED color by passing in one of the
    // RevBlinkinLedDriver.BlinkinPattern.* colors.
    public OdometryUpdByVuforia ledColor(RevBlinkinLedDriver.BlinkinPattern color) {
        m_ledColor = color;
        return this;
    }

    public OdometryUpdByVuforia maxTime(double maxTime) {
        m_Maxtime = maxTime;
        return this;
    }

    // If you don't expect the robot's position to be off by more than maybe a foot,
    // then set this to true to prevent wildly wrong Vuforia positions from being used.
    public OdometryUpdByVuforia shouldBeClose(boolean close) {
        m_shouldBeClose = close;
        return this;
    }

    // Set the number of readings that need to be consistent before the Vuforia position
    // is used.
    public OdometryUpdByVuforia numSimilar(int number) {
        m_numSimilar = number;
        return this;
    }

    @Override
    public void initialize() {
        m_runtime.reset();
    }

    @Override
    public void execute() {
        if (m_VuforiaNav.seeTarget()) {
            VectorF vectorF = m_VuforiaNav.getRobotLocationFtclib();
            double heading = m_VuforiaNav.getHeadingFtclib();
            double x = vectorF.get(0);
            double y = vectorF.get(1);
            m_poseArray.add(new Pose2d(x, y, new Rotation2d(Math.toRadians(heading))));
        }

        telemetry();
    }

    @Override
    public boolean isFinished() {
        final double maxDiff = 1.0 * (m_numSimilar - 1);

        if (m_poseArray.size() >= m_numSimilar) {

            // Look at the last m_numSimilar readings.  If they are all only a small amount
            // different, then use that pose.  Keep reading until the readings settle
            // down.
            double diffX = 0;
            double diffY = 0;
            double diffH = 0;
            int index = m_poseArray.size() - 1;
            for (int i = m_poseArray.size() - m_numSimilar; i < m_poseArray.size() - 1; i++) {
                diffX += Math.abs(m_poseArray.get(index).getX() - m_poseArray.get(i).getX());
                diffY += Math.abs(m_poseArray.get(index).getY() - m_poseArray.get(i).getY());
                diffH += Math.abs(m_poseArray.get(index).getHeading() - m_poseArray.get(i).getHeading());
            }

            if (diffX < maxDiff && diffY < maxDiff && diffH < maxDiff) {
                diffX = Math.abs(m_poseArray.get(index).getX() - m_Odometry.getX());
                diffY = Math.abs(m_poseArray.get(index).getY() - m_Odometry.getY());
                diffH = Math.abs(m_poseArray.get(index).getHeading() - m_Odometry.getHeading());
                if (!m_shouldBeClose || (diffX < 18 && diffY < 18 && diffH < 20)) {
                    m_Odometry.m_lastVufPose = m_poseArray.get(index);
                    m_Odometry.updatePose(m_poseArray.get(index));
                    // m_LED.pattern(m_ledColor).duration(2000).blink(500, 200);
                    m_poseArray.clear();
                    return true;
                }
            }
        }

        // If Vuforia never returns consistent results, give up and don't update the odometry pose.
        if (m_runtime.milliseconds() > m_Maxtime) {
            m_poseArray.clear();
            return true;
        }

        return false;
    }

    public void telemetry() {
        if (m_poseArray.size() > 0)
            m_Odometry.m_baseRobot.telemetry.addData("UpdVuf: ", "size: %d, ms = %.1f, pose: (%.1f, %.1f, %.1f)",
                                         m_poseArray.size(), m_runtime.milliseconds(),
                                         m_poseArray.get(m_poseArray.size() - 1).getX(),
                                         m_poseArray.get(m_poseArray.size() - 1).getY(),
                                         m_poseArray.get(m_poseArray.size() - 1).getHeading());
    }

}
