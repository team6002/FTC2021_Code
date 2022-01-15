package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_TEA_ElbowSetScore extends CommandBase  {
    private final SUB_TEA_elbow m_Elbow;

    public CMD_TEA_ElbowSetScore(SUB_TEA_elbow p_subsystem) {
        m_Elbow = p_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Elbow);
    }

    @Override
    public void initialize() {
        m_Elbow.setStateScore();
    }
    
    @Override
    public boolean isFinished() {
        return !m_Elbow.getIsMoving();
    }  
  
}
