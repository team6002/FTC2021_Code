package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_CarouselSetForward extends CommandBase {
    private final SUB_Carousel m_subsystem;
    
    public CMD_CarouselSetForward(SUB_Carousel subsystem) {
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
