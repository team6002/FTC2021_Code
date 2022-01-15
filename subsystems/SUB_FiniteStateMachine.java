package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;

public class SUB_FiniteStateMachine extends TriggerSubsystemBase {

    private BaseRobot m_baseRobot;

    public enum RobotState {
        HOME
        , INTAKE
        , CAPTURE
        , LEVEL1
        , LEVEL2
        , LEVEL3
        , SCORE
        , READY2SCORE
    }

    private RobotState m_currentRobotState = RobotState.CAPTURE;

    public SUB_FiniteStateMachine(BaseRobot p_baseRobot) {
        m_baseRobot = p_baseRobot;
        m_currentRobotState = RobotState.HOME;
    }

    public void setRobotState(RobotState p_robotState) {
        m_currentRobotState = p_robotState;
    }
    
    public RobotState getRobotState() {
        return m_currentRobotState;
    }
    
    public boolean getState(RobotState p_robotState) {
        return (m_currentRobotState == p_robotState);
    }

    public boolean getState(String p_robotState) {
        return (m_currentRobotState.toString().equals(p_robotState));
    }


    public void telemetry() {
        m_baseRobot.telemetry.addData("State",m_currentRobotState.name());
    }

    public void periodic() {
        telemetry();
    }    
}

