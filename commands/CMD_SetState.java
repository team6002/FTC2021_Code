package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_SetState extends CommandBase {
    private final SUB_FiniteStateMachine m_FiniteStateMachine;
    private final SUB_FiniteStateMachine.RobotState m_robotState;
    
    public CMD_SetState(SUB_FiniteStateMachine p_FiniteStateMachine, SUB_FiniteStateMachine.RobotState p_robotState) {
        m_FiniteStateMachine = p_FiniteStateMachine;
        m_robotState = p_robotState;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_FiniteStateMachine);
    }

    @Override
    public void initialize() {
        m_FiniteStateMachine.setRobotState(m_robotState);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
