package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_IntakeOnForward extends CommandBase {
    private final SUB_Intake m_subsystem;
    
    public CMD_IntakeOnForward(SUB_Intake subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setForward();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
