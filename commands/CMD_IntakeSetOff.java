package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;

/**
 * An example command that uses an example subsystem.
 */
public class CMD_IntakeSetOff extends CommandBase {
    private final SUB_Intake m_subsystem;
    
    public CMD_IntakeSetOff(SUB_Intake subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setOff();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
