package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
// import org.firstinspires.ftc.teamcode.Constants.OdomLiftConstants;

public class SUB_OdomLift extends SubsystemBase  {

    private BaseRobot m_baseRobot;
    private final Servo m_OdomLiftServo;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean m_isMoving = false;
    private double m_Offset;
    // State: 0 = home 1 = raised
    private int m_currentState;

    public SUB_OdomLift(BaseRobot p_baseRobot, final String p_OdomLiftServo, final boolean p_Reverse, double p_Offset) {
        m_baseRobot = p_baseRobot;
        m_Offset = p_Offset;
        m_OdomLiftServo = m_baseRobot.hardwareMap.servo.get(p_OdomLiftServo);
        if (p_Reverse) {
            m_OdomLiftServo.setDirection(Servo.Direction.REVERSE);
        } else { m_OdomLiftServo.setDirection(Servo.Direction.FORWARD); };
    }
    
    public void setStateHome() {
        m_OdomLiftServo.setPosition(.1 + m_Offset);
        if (m_currentState != 0) {
            m_currentState = 0;
            m_isMoving = true;
            runtime.reset();
        }
    }
    
    public void setStateRaised() {
        m_OdomLiftServo.setPosition(.72 + m_Offset);
        if (m_currentState != 1) {
            m_currentState = 1;
            m_isMoving = true;
            runtime.reset();
        }
    }

    public void toggleLift() {
        if (m_currentState == 0) setStateRaised();
        else setStateHome();
    }


    public boolean isMoving() {
        if (m_isMoving) {
            if (200 < runtime.milliseconds()) {
                m_isMoving = false;
            }
        }
        
        return m_isMoving;
    }
    
    // public void telemetry() {

    // }

    // public void periodic() {        

    // }
    
}    
    
