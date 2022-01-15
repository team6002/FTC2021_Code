package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.ftclib.purepursuit.Path;
import org.firstinspires.ftc.teamcode.ftclib.purepursuit.PathMotionProfile;
import org.firstinspires.ftc.teamcode.ftclib.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.ftclib.purepursuit.waypoints.StartWaypoint;
import org.firstinspires.ftc.teamcode.ftclib.purepursuit.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.ftclib.util.*;

public class PurePursuit  extends CommandBase {

    private Drivetrain m_drivetrain;
    private Odometry m_odometry;
    private Path m_path;

    public PurePursuit(Drivetrain drivetrain, Odometry odometry, Waypoint... waypoints) {
        m_drivetrain = drivetrain;
        m_odometry = odometry;

        m_path = new Path();
        loadPath(waypoints);
        m_path.disableRetrace();

        addRequirements(m_drivetrain , m_odometry);

    }

    @Override
    public void initialize() {
        m_path.init();
    }


    public void loadPath(Waypoint... waypoints) {
        double robotX, robotY;
        robotX = m_odometry.getX();
        robotY = m_odometry.getY();
        // add robot current position as the StartPoint
        addWaypoint(new StartWaypoint(robotX,robotY));
        // add the list of waypoints
        addWaypoints(waypoints);
    }

    public void addWaypoint(Waypoint waypoint) {
        m_path.add(waypoint);
    }

    public void addWaypoints(Waypoint... waypoints) {
        for (Waypoint waypoint : waypoints) this.addWaypoint(waypoint);
    }

    public void removeWaypointAtIndex(int index) {
        m_path.remove(index);
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        Pose2d robotPose = m_odometry.getPose();
        double[] motorSpeeds = m_path.loop(robotPose.getTranslation().getX(), robotPose.getTranslation().getY(), robotPose.getHeading());
        m_drivetrain.driveRobotCentric(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2]);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return m_path.isFinished();
    }
}
