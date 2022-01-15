package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CMD_DriveMoveTime extends CommandBase {
    private final Drivetrain m_subsystem;
    
    private ElapsedTime runtime;
    private boolean m_isFinished = false;
    private double m_Milliseconds;
    private double m_strafe=0;
    private double m_speed=0;
    private double m_turn=0;
    
    public CMD_DriveMoveTime(Drivetrain subsystem, double p_Milliseconds){
        m_Milliseconds = p_Milliseconds;
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        
    }

    public CMD_DriveMoveTime strafe(double p_strafe) {
        m_strafe = p_strafe;
        return this;
    }

    public CMD_DriveMoveTime speed(double p_speed) {
        m_speed = p_speed;
        return this;
    }
    
    public CMD_DriveMoveTime turn(double p_turn) {
        m_turn = p_turn;
        return this;
    }    

    @Override
    public void initialize() {
        m_isFinished = false;
        runtime = new  ElapsedTime();
        runtime.reset();
        m_subsystem.driveRobotCentric(m_strafe,m_speed,m_turn);
    }
    
    @Override
    public void execute(){
     if (m_Milliseconds < runtime.milliseconds()){
         m_isFinished = true;
      }
    }
    
    @Override
    public void end(boolean interrupted){
        m_subsystem.stop();
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }  
  
}
