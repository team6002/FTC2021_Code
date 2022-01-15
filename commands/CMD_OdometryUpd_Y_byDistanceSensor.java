package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Rotation2d;


public class CMD_OdometryUpd_Y_byDistanceSensor extends CommandBase {
    private final Odometry m_Odometry;
    private Sensor_RangeSensor m_rangeSensor;
    private double m_X, m_Y, m_heading;
    private double m_redSideModifier;



    public CMD_OdometryUpd_Y_byDistanceSensor(Odometry p_odometry
        , Sensor_RangeSensor p_rangeSensor
        , double p_redSideModifier
        , double p_X, double p_heading) {
        m_Odometry = p_odometry;
        m_rangeSensor = p_rangeSensor;
        m_X=p_X;
        m_heading = p_heading;
        m_redSideModifier = p_redSideModifier;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(m_FiniteStateMachine);
    }

    @Override
    public void initialize() {
        
        m_Y = m_rangeSensor.getDistance();
        // valve are only valid up 50in away
        if (m_Y < 50) {
            m_Y = (72.5 - m_Y) * m_redSideModifier; 
            m_Odometry.updatePose(new Pose2d(m_X, m_Y, new Rotation2d(Math.toRadians(m_heading))));

        }
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
