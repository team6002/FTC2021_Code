package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CMD_DriveIntake extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final SUB_Intake m_intake;
    private final SUB_Sensor_IRBucket m_Sensor_IRBucket;
    
    private ElapsedTime runtime;
    private boolean m_isFinished = false;
    private double m_Milliseconds;
    private double m_strafe=0;
    private double m_speed=0;
    private double m_turn=0;
    private double m_cycleTime = 400;
    private boolean m_backup=false;
    private int m_counter=0;
    private int m_maxTries =5;
    
    public CMD_DriveIntake(Drivetrain p_drivetrain
        , SUB_Intake p_intake
        , SUB_Sensor_IRBucket p_Sensor_IRBucket
        , int p_maxTries) {
            
        m_drivetrain = p_drivetrain;
        m_intake=p_intake;
        m_maxTries = p_maxTries;
        m_Sensor_IRBucket = p_Sensor_IRBucket;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drivetrain);
        
    }

    public CMD_DriveIntake strafe(double p_strafe) {
        m_strafe = p_strafe;
        return this;
    }

    public CMD_DriveIntake speed(double p_speed) {
        m_speed = p_speed;
        return this;
    }
    
    public CMD_DriveIntake turn(double p_turn) {
        m_turn = p_turn;
        return this;
    } 

    @Override
    public void initialize() {
        m_isFinished = false;
        m_backup=false;
        m_counter = 0;
        runtime = new  ElapsedTime();
        runtime.reset();
        m_drivetrain.driveRobotCentric(m_strafe,m_speed,m_turn);
    }
    
    @Override
    public void execute(){
        if (m_Sensor_IRBucket.getState())
        {
            m_intake.setOff();
            m_isFinished = true;
            return;
        }
        
        
        
        if (m_counter > m_maxTries) m_isFinished = true;
        
        if (1000 < runtime.milliseconds())
        {
            m_intake.setForward();
            runtime.reset();
            m_counter +=1;
        } else if (950 < runtime.milliseconds())
        {
            m_intake.setReverse();
            
        }else if (650 < runtime.milliseconds())
        {
            m_drivetrain.driveRobotCentric(m_strafe,m_speed,m_turn);
        } else 
        {
            m_drivetrain.driveRobotCentric(0,.2,0);
        }
    }
    
    @Override
    public void end(boolean interrupted){
        m_drivetrain.stop();
    }
    
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }  
  
}
