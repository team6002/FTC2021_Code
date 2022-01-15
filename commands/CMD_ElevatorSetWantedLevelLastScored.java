package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CMD_ElevatorSetWantedLevelLastScored extends CommandBase {
    // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SUB_Elevator m_elevator;
    SUB_FiniteStateMachine m_FiniteStateMachine;
    private int m_wantedLevel;
    private ElapsedTime runtime = new  ElapsedTime();
    private boolean m_isFinished = false;
    private double m_timeout = 3000;
    private boolean m_waitForCompletion = true;

    // @param subsystem The subsystem used by this command.
    public CMD_ElevatorSetWantedLevelLastScored(SUB_FiniteStateMachine p_FiniteStateMachine
        ,SUB_Elevator subsystem) {
            
        m_FiniteStateMachine = p_FiniteStateMachine;    
        m_elevator = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    public CMD_ElevatorSetWantedLevelLastScored timeout(double timeout) {
        m_timeout = timeout;
        return this;
    }     

    @Override
    public void initialize() {
        m_isFinished = false;
        runtime.reset();        
        m_wantedLevel = m_elevator.setWantedLevelLastScored();
    }

   @Override
    public void execute(){
        
        if (!m_waitForCompletion) m_isFinished = true;
        else if (m_timeout < runtime.milliseconds()) 
            { 
                // something wrong can't get to wanted level.
                // goto level 1
                m_wantedLevel = 1;
                m_elevator.setWantedLevel(m_wantedLevel);
                m_isFinished = true;
            }
        else m_isFinished = (m_elevator.getLevel() == m_wantedLevel);
    }
    
    @Override
    public void end(boolean interrupted) {
        switch (m_wantedLevel) 
        {
            case 1 :    m_FiniteStateMachine.setRobotState(SUB_FiniteStateMachine.RobotState.LEVEL1);
                break;
            case 2 :    m_FiniteStateMachine.setRobotState(SUB_FiniteStateMachine.RobotState.LEVEL2);
                break;
            case 3 :    m_FiniteStateMachine.setRobotState(SUB_FiniteStateMachine.RobotState.LEVEL3);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
