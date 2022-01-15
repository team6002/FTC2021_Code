package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;

public class GlobalVariables {

    private boolean m_displayTelemetry = true;

    private GlobalVariables () {

    }

    private static GlobalVariables m_Instance = null;

    public synchronized static GlobalVariables getInstance() {
        if (m_Instance == null) {
            m_Instance = new GlobalVariables();
        }
        return m_Instance;
    }

    private Pose2d m_robotPose = new Pose2d(0, 0, new Rotation2d(0));

    public void setRobotPose(Pose2d p_pose) {
        m_robotPose = p_pose;
    }

    public Pose2d getRobotPose() {
        return m_robotPose;
    }

    public boolean getDisplayTelemetry() {
        return m_displayTelemetry;
    }
    

    
}
