package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;



public class CMD_ElevatorSetWantedLevel extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SUB_Elevator m_elevator;
    private final int m_wantedLevel;
    private ElapsedTime runtime = new  ElapsedTime();
    private boolean m_isFinished = false;
    private double m_timeout = 3000;
    private boolean m_waitForCompletion = true;


    // @param subsystem The subsystem used by this command.
    public CMD_ElevatorSetWantedLevel(SUB_Elevator subsystem, int p_wantedLevel) {
        m_elevator = subsystem;
        m_wantedLevel = p_wantedLevel;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    public CMD_ElevatorSetWantedLevel timeout(double timeout) {
        m_timeout = timeout;
        return this;
    }    
    
    public CMD_ElevatorSetWantedLevel noWait() {
        m_waitForCompletion= false;
        return this;
    }    

    @Override
    public void initialize() {
        m_isFinished = false;
        runtime.reset();
        m_elevator.setWantedLevel(m_wantedLevel);
    }
    
    @Override
    public void execute(){
        
        if (!m_waitForCompletion) m_isFinished = true;
        else if (m_timeout < runtime.milliseconds()) 
            { 
                // something wrong can't get to wanted level.
                // goto level 1
                m_elevator.setWantedLevel(1);
                m_isFinished = true;
            }
        else m_isFinished = (m_elevator.getLevel() == m_wantedLevel);
    }
    

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
