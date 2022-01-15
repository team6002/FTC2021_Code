package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_IntakeSetSpeed extends CommandBase {
    private final SUB_Intake m_subsystem;
    private double m_speed;
    
    public CMD_IntakeSetSpeed(SUB_Intake subsystem,double p_speed) {
        m_subsystem = subsystem;
        m_speed = p_speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        m_subsystem.setForward(m_speed);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
