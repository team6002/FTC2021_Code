package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants.ElbowConstants;



public class SUB_TEA_elbow extends SubsystemBase {
    
    private BaseRobot m_baseRobot;
    
    private final DcMotor m_elbowMotor;
    
    private boolean m_isMoving = false;
    private int m_elbowState; // 0=home, 1=intake, 2=capture, 3=ready2score, 4=score
    private int m_elbowWantedState;
    private double m_previousWantedEncoderPosition;
    
    public SUB_TEA_elbow(BaseRobot p_baseRobot, final String p_elbowMotor) {
        m_baseRobot = p_baseRobot;
        m_elbowMotor = m_baseRobot.hardwareMap.get(DcMotor.class,p_elbowMotor);
        m_elbowMotor.setDirection(DcMotor.Direction.REVERSE);
        
        m_elbowWantedState=0;
        m_previousWantedEncoderPosition=-1;
    }
    
    public void resetEncoder() {
        m_elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }    

    public void setStateReadyToPickUp() {
        setWantedState(1);   
    }
    
    public void setStateReadyToScore() {
        setWantedState(3);   
    }
    
    public void setStateScore() {
        setWantedState(4);   
    }    
    
    public void setStateHome() {
        setWantedState(0);   
    }    

    public void setWantedState(int p_wantedState) {
        m_isMoving = true;
        m_elbowWantedState = p_wantedState;
    }

    public int getState() {
        int state=-999;
        
        if (!m_isMoving) state = m_elbowWantedState;
        
        return state;
    }

    public double getElbowPosition() {
        return m_elbowMotor.getCurrentPosition();
    }
 
    public boolean getIsMoving() {
        return m_isMoving;
    }
 
    @Override
    public void periodic() {
         // we need to determine the encoder position for the level we want
        int m_elbowWantedEncoderPosition=0;
        double power = ElbowConstants.kMotorPower;
        
        switch(m_elbowWantedState) {
            case 0 : m_elbowWantedEncoderPosition = 0; // home
                        power = .30;
                break;
            case 1 : m_elbowWantedEncoderPosition = 990; //1014; //765; // ready2 pick-up
                        power = .35;
                break;
            case 2 : m_elbowWantedEncoderPosition = 230; //264; //200; // not used
                break;
            case 3 : // ready to score
                    m_elbowWantedEncoderPosition = 510; // 536; //570; //430; // ready 2 score
                break;
            case 4 : m_elbowWantedEncoderPosition = 595; //629; //475; // score
                break;
        }
        
        // check if elbow is at the wanted position
        if ( Math.abs(m_elbowWantedEncoderPosition - getElbowPosition()) < 30) {
            // if yes, elbow is at the wanted state
            if (m_isMoving) m_isMoving = false; // indicate the elbow has stopped moving;
        } else {
            // if no, elbow is not at the wanted state
            if (!m_isMoving) m_isMoving = true; // indicate the elbow is moving;

            // check if the previous set position is equal to the wanted position
            if ((m_previousWantedEncoderPosition - m_elbowWantedEncoderPosition) == 0) {
                // already set, just wait
            } else {
                // if not equal, set the new wanted position
                
                m_elbowMotor.setTargetPosition(m_elbowWantedEncoderPosition);
                m_elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m_elbowMotor.setPower(power);
                
                // remember the last position
                m_previousWantedEncoderPosition = m_elbowWantedEncoderPosition; 
                
            }
        }

        telemetry();
    }

    public void telemetry() {

        m_baseRobot.telemetry.addData("Elbow:","WantedState %d - IsMoving [%b] Enc[%.0f]"
            ,m_elbowWantedState,m_isMoving,getElbowPosition());

    }
    

}
