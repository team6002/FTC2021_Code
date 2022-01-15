package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_TEA_ClawSetClose extends CommandBase  {
    private final SUB_TEA_claw m_Claw;

    public CMD_TEA_ClawSetClose(SUB_TEA_claw p_subsystem) {
        m_Claw = p_subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_Claw);
    }

    @Override
    public void initialize() {
        m_Claw.setStateClose();
    }
    
    @Override
    public boolean isFinished() {
        return !m_Claw.isMoving();
    }  
  
}
