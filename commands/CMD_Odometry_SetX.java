package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;


public class CMD_Odometry_SetX extends CommandBase {
    private final Odometry m_Odometry;
    private double m_X, m_Y, m_heading;



    public CMD_Odometry_SetX(Odometry p_odometry, double p_X) {
        m_Odometry = p_odometry;
        m_X=p_X;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        m_Odometry.updatePose(new Pose2d(m_X,m_Odometry.getY(), new Rotation2d(m_Odometry.getHeading())));

    }

    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
