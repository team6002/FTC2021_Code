package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Intake;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CMD_IntakeSetReverse extends CommandBase {
    private final SUB_Intake m_subsystem;
    private double m_milliseconds;
    private ElapsedTime runtime;
    private boolean m_isFinished = false;
    
    public CMD_IntakeSetReverse(SUB_Intake subsystem) {
        m_subsystem = subsystem;
        // m_milliseconds = p_milliseconds;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    
    

    public void initialize() {
        m_isFinished = false;
        m_subsystem.setReverse();
        runtime = new  ElapsedTime();
        runtime.reset();
        
    }
    
    @Override
    public void execute(){
     if (m_milliseconds < runtime.milliseconds()){
         m_isFinished = true;
      }
      m_isFinished = true;
    }
    
    @Override
    public void end(boolean interrupted){
    //  m_subsystem.setOff();
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }  
  
}
