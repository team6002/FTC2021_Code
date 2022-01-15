package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.*;


public class CMD_TEA_SetState extends CommandBase {
    private final FSM_TeamElementArm m_FSM;
    private final FSM_TeamElementArm.State m_State;
    
    public CMD_TEA_SetState(FSM_TeamElementArm p_FSM, FSM_TeamElementArm.State p_State) {
        m_FSM = p_FSM;
        m_State = p_State;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_FSM);
    }

    @Override
    public void initialize() {
        m_FSM.setState(m_State);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
