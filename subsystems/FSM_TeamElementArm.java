package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;

public class FSM_TeamElementArm extends TriggerSubsystemBase {

    private BaseRobot m_baseRobot;
    private String m_name = "TEA";

    public enum State {
        HOME
        , READY2SCORE
        , READY2PICKUP
        , CAPTURED
        , SCORE
        , SCORE_RELEASE
    }

    private State m_currentState = State.HOME;

    public FSM_TeamElementArm(BaseRobot p_baseRobot) {
        m_baseRobot = p_baseRobot;
        m_currentState = State.HOME;
    }

    public void setState(State p_State) {
        m_currentState = p_State;
    }
    
    public State getState() {
        return m_currentState;
    }
    
    public boolean getState(State p_State) {
        return (m_currentState == p_State);
    }

    public boolean getState(String p_State) {
        return (m_currentState.toString().equals(p_State));
    }


    public void telemetry() {
        m_baseRobot.telemetry.addData(m_name+"[State]",m_currentState.name());
    }

    public void periodic() {
        telemetry();
    }    
}

