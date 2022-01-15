package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_CarouselToggleForwardOff extends CommandBase {
    private final SUB_Carousel m_subsystem;
    
    public CMD_CarouselToggleForwardOff(SUB_Carousel subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.toggleForwardOff();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
