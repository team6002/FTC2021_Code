package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CMD_Drivetrain_SetControlToggle extends CommandBase {
    private final Drivetrain m_subsystem;
    
    private ElapsedTime runtime;
    private boolean m_isFinished = true;
    private double m_Milliseconds;
    private double m_strafe=0;
    private double m_speed=0;
    private double m_turn=0;
    
    public CMD_Drivetrain_SetControlToggle(Drivetrain subsystem){
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        
    }

    @Override
    public void initialize() {
        m_subsystem.setControlToggle();
    }
    
    @Override
    public void execute(){

    }
    
    @Override
    public void end(boolean interrupted){
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }  
  
}
