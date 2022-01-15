package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.WristConstants;

public class SUB_TEA_wrist extends SubsystemBase  {

    private BaseRobot m_baseRobot;
    private final Servo m_WristServo;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean m_isMoving = false;
    private double m_moveTime=0;

    // State: 0 = home, 1=intake, 2=capture, 3= score
    private int m_currentState=2;

    public SUB_TEA_wrist(BaseRobot p_baseRobot, final String p_wristServo) {
        m_baseRobot = p_baseRobot;
        m_WristServo = m_baseRobot.hardwareMap.servo.get(p_wristServo);
        m_WristServo.setDirection(Servo.Direction.REVERSE);
        m_currentState = 2;
    }
    
    public void setStateCapture() {
        m_WristServo.setPosition(.42 + WristConstants.kServoOffset);
        if (m_currentState != 2) {
            m_currentState = 2;
            m_isMoving = true;
            m_moveTime = 100;
            runtime.reset();
        }
    }
    
    public void setStateHome() {
        m_WristServo.setPosition(.13 + WristConstants.kServoOffset);
        if (m_currentState != 0) {
            m_currentState = 0;
            m_isMoving = true;
            m_moveTime = 100;
            runtime.reset();
        }
    }
    
    public void setStateReadyToPickup() {
        m_WristServo.setPosition(.70 + WristConstants.kServoOffset);
        if (m_currentState != 1) {
            if (m_currentState == 2) m_moveTime = 500; // if current state is capture, give extra time
            else m_moveTime = 300;
    
            m_isMoving = true;
            m_currentState = 1;
            runtime.reset();
        }
    }
    
    public void setStateScore() {
        m_WristServo.setPosition(.35 + WristConstants.kServoOffset);
        if (m_currentState != 3) {
            m_currentState = 3;
            m_isMoving = true;
            m_moveTime = 100;
            runtime.reset();
        }
    }

    public void setStateReadyToScore() {
        m_WristServo.setPosition(.38 + WristConstants.kServoOffset);
        if (m_currentState != 3) {
            m_currentState = 3;
            m_isMoving = true;
            m_moveTime = 100;
            runtime.reset();
        }
    }

    public boolean isMoving() {
        if (m_isMoving) {
            if (m_moveTime < runtime.milliseconds()) {
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
    
