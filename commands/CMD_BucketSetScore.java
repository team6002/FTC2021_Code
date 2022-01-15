package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_BucketSetScore extends CommandBase  {
    private final SUB_Bucket m_Bucket;

    public CMD_BucketSetScore(SUB_Bucket p_subsystem) {
        m_Bucket = p_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Bucket);
    }

    @Override
    public void initialize() {
        m_Bucket.setStateScore();
    }
    
    @Override
    public boolean isFinished() {
        return !m_Bucket.isMoving();
    }  
  
}
