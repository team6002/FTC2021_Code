package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_TEA_WristSetScore extends CommandBase  {
    private final SUB_TEA_wrist m_Wrist;

    public CMD_TEA_WristSetScore(SUB_TEA_wrist p_subsystem) {
        m_Wrist = p_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Wrist);
    }

    @Override
    public void initialize() {
        m_Wrist.setStateScore();
    }
    
    @Override
    public boolean isFinished() {
        return !m_Wrist.isMoving();
    }  
  
}
