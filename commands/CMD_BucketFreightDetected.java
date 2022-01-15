package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CMD_BucketFreightDetected extends CommandBase{

    private final SUB_Sensor_IRBucket m_Sensor_IRBucket;
    private boolean m_isFinished = false;

    public CMD_BucketFreightDetected(SUB_Sensor_IRBucket p_Sensor_IRBucket) {
        m_Sensor_IRBucket = p_Sensor_IRBucket;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(m_Sensor_IRBucket);
        
    }    
    

    @Override
    public void initialize() {
        m_isFinished = false;
    }
    
    @Override
    public void execute(){
        m_isFinished = m_Sensor_IRBucket.getState();
    }
    
    @Override
    public void end(boolean interrupted){
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }  
    
    

}
