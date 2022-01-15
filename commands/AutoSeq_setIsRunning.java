package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class AutoSeq_setIsRunning extends CommandBase  {
    private final AutoSequencer m_AutoSequencer;
    private boolean m_isRunning;

    public AutoSeq_setIsRunning(AutoSequencer p_AutoSequencer, boolean p_isRunning) {
        m_AutoSequencer = p_AutoSequencer;
        m_isRunning = p_isRunning;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(m_Bucket);
    }

    @Override
    public void initialize() {
        m_AutoSequencer.setIsRunning(m_isRunning);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
