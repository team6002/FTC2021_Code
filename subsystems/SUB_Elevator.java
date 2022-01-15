package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants.ElevatorConstants;



public class SUB_Elevator extends SubsystemBase {
    
    private BaseRobot m_baseRobot;
    
    private final DcMotor m_elevatorMotor;

    private int m_previousScoreLevel = 1;
    
    private boolean m_isMoving = false;
    private int m_elevatorLvl; // elevator level start at Zero(0) Home position
    private int m_elevatorWantedLvl;
    private double m_previousWantedEncoderPosition;
    
    public SUB_Elevator(BaseRobot p_baseRobot, final String p_ElevatorMotor) {
        m_baseRobot = p_baseRobot;
        m_elevatorMotor = m_baseRobot.hardwareMap.get(DcMotor.class,p_ElevatorMotor);
        m_elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // m_elevatorLvl= 0; 
        m_elevatorWantedLvl=0;
        m_previousScoreLevel = 1;
        m_previousWantedEncoderPosition=-1;
    }

    public void resetEncoder() {
        m_elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setWantedLevel(int p_wantedLevel) {
        m_isMoving = true;
        m_elevatorWantedLvl = p_wantedLevel;
    }

    public int setWantedLevelLastScored() {
        m_isMoving = true;
        m_elevatorWantedLvl = m_previousScoreLevel;
        
        return m_previousScoreLevel;
    }

    public int getLevel() {
        int level=-999;
        
        if (!m_isMoving) level = m_elevatorWantedLvl;
        
        return level;
    }

    public double getElevatorPosition() {
        return m_elevatorMotor.getCurrentPosition();
    }
 
    public boolean getIsMoving() {
        return m_isMoving;
    }
 
    public void setLastScoreLevel() {
        m_previousScoreLevel = m_elevatorWantedLvl;
    }
 
    public void periodic() {
         // we need to determine the encoder position for the level we want
        int m_elevatorWantedPosition=0;
        double power = ElevatorConstants.kMotorPower;
        
        switch(m_elevatorWantedLvl) {
            case 0 : m_elevatorWantedPosition = 0;
                break;
            case 1 : m_elevatorWantedPosition = 0;
                break;
            case 2 : m_elevatorWantedPosition = 200;
                break;
            case 3 : m_elevatorWantedPosition = 430;
                break;
        }
        
        // check if elevator is at the wanted position
        if ( Math.abs(m_elevatorWantedPosition - getElevatorPosition()) < 30) {
            // if yes, elevator is at the wanted level
            if (m_isMoving) m_isMoving = false; // indicate the elevator has stopped;
        } else {
            // if no, elevator is not at the wanted level
            if (!m_isMoving) m_isMoving = true; // indicate the elevator is moving;

            // check if the previous set position is equal to the wanted position
            if ((m_previousWantedEncoderPosition - m_elevatorWantedPosition) == 0) {
                // already set, just wait
            } else {
                // if not equal, set the new wanted position
                
                m_elevatorMotor.setTargetPosition(m_elevatorWantedPosition);
                m_elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_elevatorMotor.setPower(power);
                
                // remember the last position
                m_previousWantedEncoderPosition = m_elevatorWantedPosition; 
                
            }
        }

        telemetry();
    }

    public void telemetry() {

        m_baseRobot.telemetry.addData("Elevator:","WantedLvl %d - IsMoving [%b] Enc[%.0f]"
            ,m_elevatorWantedLvl,m_isMoving,getElevatorPosition());

    }
    

}
