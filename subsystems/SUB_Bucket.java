package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.BucketConstants;

public class SUB_Bucket extends SubsystemBase  {

    private BaseRobot m_baseRobot;
    private final Servo m_BucketServo;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean m_isMoving = false;
    private double m_moveTime=0;

    // State: 0 = home, 1=intake, 2=capture, 3= score
    private int m_currentState=2;

    public SUB_Bucket(BaseRobot p_baseRobot, final String p_bucketServo) {
        m_baseRobot = p_baseRobot;
        m_BucketServo = m_baseRobot.hardwareMap.servo.get(p_bucketServo);
        m_BucketServo.setDirection(Servo.Direction.REVERSE);
        m_currentState = 2;
    }
    
    public void setStateCapture() {
        m_BucketServo.setPosition(.42 + BucketConstants.kServoOffset);
        if (m_currentState != 2) {
            m_currentState = 2;
            m_isMoving = true;
            m_moveTime = 300;
            runtime.reset();
        }
    }
    
    public void setStateCheckDuck() {
        m_BucketServo.setPosition(.30 + BucketConstants.kServoOffset);
        if (m_currentState != 2) {
            m_currentState = 2;
            m_isMoving = true;
            m_moveTime = 300;
            runtime.reset();
        }
    }

    public void setStateHome() {
        m_BucketServo.setPosition(.13 + BucketConstants.kServoOffset);
        if (m_currentState != 0) {
            m_currentState = 0;
            m_isMoving = true;
            m_moveTime = 300;
            runtime.reset();
        }
    }
    
    
    public void setStateIntake() {
        m_BucketServo.setPosition(.085 + BucketConstants.kServoOffset);
        if (m_currentState != 1) {
            if (m_currentState == 2) m_moveTime = 500; // if current state is capture, give extra time
            else m_moveTime = 200;
    
            m_isMoving = true;
            m_currentState = 1;
            runtime.reset();
        }
    }
    
    public void setStateScore() {
        m_BucketServo.setPosition(.69 + BucketConstants.kServoOffset);
        if (m_currentState != 3) {
            m_currentState = 3;
            m_isMoving = true;
            m_moveTime = 300;
            runtime.reset();
        }
    }

    public void setStateReadyToScore() {
        m_BucketServo.setPosition(.59 + BucketConstants.kServoOffset);
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
    
