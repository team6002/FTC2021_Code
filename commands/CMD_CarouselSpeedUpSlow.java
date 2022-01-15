package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SUB_Carousel;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CMD_CarouselSpeedUpSlow extends CommandBase {
    private final SUB_Carousel m_subsystem;
    
    private ElapsedTime runtime;
    private boolean m_isFinished = false;
    private double m_Milliseconds;
    
    
    public CMD_CarouselSpeedUpSlow(SUB_Carousel subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_Milliseconds = 1400;
        // m_subsystem.setForward();
        runtime = new  ElapsedTime();
        runtime.reset();
        
    }
    
    @Override
    public void execute(){
     if (m_Milliseconds < runtime.milliseconds()){
         m_isFinished = true;
      }
  
      double power = .7 + (0.00025 * runtime.milliseconds());
    //   double power = .21 + (0.0006 * runtime.milliseconds());
      
        // if (power > 0.9){
        //     power = 0.9;
        // } 

      m_subsystem.setForward(power);
    //   m_subsystem.setForward(.7);
      
    }
    
    @Override
    public void end(boolean interrupted){
     m_subsystem.setOff();
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }  
  
}
