package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SUB_Intake extends SubsystemBase {

    private BaseRobot m_baseRobot;

    private final DcMotor m_intakeMotor;

    private int m_currentState = IntakeConstants.kState_Off;  // -1 = REVERSE, 0 = OFF, 1 = FORWARD

    private boolean m_isReverseTriggered = false;
    private int m_savedReverseTriggerState;
    
    
    

    public SUB_Intake(BaseRobot p_baseRobot, final String p_intakemotor) {
        m_baseRobot = p_baseRobot;
        m_intakeMotor = m_baseRobot.hardwareMap.get(DcMotor.class,p_intakemotor);
        m_intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        m_intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    private boolean getIntakeOn(){
        return (m_currentState != IntakeConstants.kState_Off);
    } 
    
    public boolean setForward() {
        m_intakeMotor.setPower(IntakeConstants.kMotorPower);   
        m_currentState = IntakeConstants.kState_Forward;
        return true;
    }
 
     public boolean setForward(double p_speed) {
        m_intakeMotor.setPower(p_speed);   
        m_currentState = IntakeConstants.kState_Forward;
        return true;
    }
    
    public boolean setReverse() {
        m_intakeMotor.setPower(-IntakeConstants.kMotorPower);   
        m_currentState = IntakeConstants.kState_Reverse;
        return true;
    }
    
    public void setReverseTriggered(double p_level) {
        m_isReverseTriggered=true;
        m_savedReverseTriggerState = m_currentState;
        setReverse();
    }

    public void setReverseTriggeredReset() {
        m_isReverseTriggered=false;
        if (m_savedReverseTriggerState == IntakeConstants.kState_Forward) setForward();
        else setOff(); 
        
    }
    
    public boolean getReverseTriggered() {
        return m_isReverseTriggered;
    }

    public boolean setReverse(double p_speed) {
        m_intakeMotor.setPower(-p_speed);   
        m_currentState = IntakeConstants.kState_Reverse;
        return true;
    }    
    
    public boolean setOff() {
        m_intakeMotor.setPower(0);
        m_currentState = IntakeConstants.kState_Off;
        return true;
    }
    
    public double getVelocity() {
        return ((DcMotorEx)m_intakeMotor).getVelocity();
    }
    
    public boolean toggleReverseOff() {
        if (m_currentState != IntakeConstants.kState_Off) setOff(); // turn OFF if not equal to OFF (0)
        else setReverse(); // REVERSE if currently OFF (0)
    
        return true;
    }
    

    public int getState() {
        return m_currentState;
    }
    
    public void telemetry() {
        m_baseRobot.telemetry.addData("Intake:","%d-%d",m_currentState, m_savedReverseTriggerState);

    }
    
    @Override
    public void periodic() {
        telemetry();

    }  
}
