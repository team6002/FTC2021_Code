package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import com.qualcomm.robotcore.hardware.DigitalChannel;


public class SUB_Sensor_IRBucket extends TriggerSensorBase {

    BaseRobot m_baseRobot;
    DigitalChannel m_irSensor;

    public SUB_Sensor_IRBucket(BaseRobot p_baseRobot, final String p_irSensor) {
        m_baseRobot = p_baseRobot;
        m_irSensor = m_baseRobot.hardwareMap.get(DigitalChannel.class, p_irSensor);    //  Use generic form of device mapping
        m_irSensor.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel

    }

    public boolean getState() {
        return m_irSensor.getState();
    }
    
    
}