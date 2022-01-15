package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Bucket;


public class CMD_BucketSetCapture extends CommandBase  {
    private final SUB_Bucket m_Bucket;

    public CMD_BucketSetCapture(SUB_Bucket p_subsystem) {
        m_Bucket = p_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Bucket);
    }

    @Override
    public void initialize() {
        m_Bucket.setStateCapture();
    }
    
    @Override
    public boolean isFinished() {
        return !m_Bucket.isMoving();
    }  
  
}
