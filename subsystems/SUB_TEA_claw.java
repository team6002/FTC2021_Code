package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.ClawConstants;

public class SUB_TEA_claw extends SubsystemBase  {

    private BaseRobot m_baseRobot;
    private final Servo m_ClawServo;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean m_isMoving = false;
    private double m_moveTime=0;

    // State: 0 = closed, 1=open,
    private int m_currentState=0;

    public SUB_TEA_claw(BaseRobot p_baseRobot, final String p_clawServo) {
        m_baseRobot = p_baseRobot;
        m_ClawServo = m_baseRobot.hardwareMap.servo.get(p_clawServo);
        m_ClawServo.setDirection(Servo.Direction.FORWARD);
        m_currentState = 0;
    }
    
    public void setStateClose() {
        m_ClawServo.setPosition(.40 + ClawConstants.kServoOffset);
        if (m_currentState != 0) {
            m_currentState = 0;
            m_isMoving = true;
            m_moveTime = 300;
            runtime.reset();
        }
    }
    
     public void setStateOpen() {
        m_ClawServo.setPosition(.10 + ClawConstants.kServoOffset);
        if (m_currentState != 1) {
            m_currentState = 1;
            m_isMoving = true;
            m_moveTime = 300;
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
    
