package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Constants.CarouselConstants;


public class SUB_Carousel extends SubsystemBase {

    private BaseRobot m_baseRobot;
    private final DcMotor m_carouselMotor;

    private int m_currentState = CarouselConstants.kState_Off;  // -1 = REVERSE, 0 = OFF, 1 = FORWARD

    public SUB_Carousel(BaseRobot p_baseRobot, final String p_carouselmotor) {
        m_baseRobot = p_baseRobot;
        m_carouselMotor = m_baseRobot.hardwareMap.get(DcMotor.class,p_carouselmotor);
        m_carouselMotor.setDirection(DcMotor.Direction.FORWARD);
        m_carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private boolean getCarouselOn(){
        return (m_currentState != CarouselConstants.kState_Off);
    } 
    
    public boolean setForward() {
        m_carouselMotor.setPower(CarouselConstants.kMotorPower);   
        m_currentState = CarouselConstants.kState_Forward;
        return true;
    }
    
    public boolean setForward(double p_Power) {
        m_carouselMotor.setPower(p_Power);   
        m_currentState = CarouselConstants.kState_Forward;
        return true;
    }
    
    public boolean setReverse() {
        m_carouselMotor.setPower(-CarouselConstants.kMotorPower);   
        m_currentState = CarouselConstants.kState_Reverse;
        return true;
    }
    
    public boolean setOff() {
        m_carouselMotor.setPower(0);
        m_currentState = CarouselConstants.kState_Off;
        return true;
    }
    
    public boolean toggleForwardOff() {
        if (m_currentState != CarouselConstants.kState_Off) setOff(); // turn OFF if not equal to OFF (0)
        else setForward(); // REVERSE if currently OFF (0)

        return true;
    }
    
    public void setBlueAlliance() {
        m_carouselMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setRedAlliance() {
        m_carouselMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    
    // public void telemetry() {

    // }

    // public void periodic() {        

    // }    
}
