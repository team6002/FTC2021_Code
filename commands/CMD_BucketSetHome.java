package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_BucketSetHome extends CommandBase  {
    private final SUB_Bucket m_Bucket;

    public CMD_BucketSetHome(SUB_Bucket p_subsystem) {
        m_Bucket = p_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Bucket);
    }

    @Override
    public void initialize() {
        m_Bucket.setStateHome();
    }
    
    @Override
    public boolean isFinished() {
        return !m_Bucket.isMoving();
    }  
  
}
