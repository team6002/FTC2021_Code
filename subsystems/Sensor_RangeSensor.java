package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Sensor_RangeSensor extends TriggerSensorBase {

    BaseRobot m_baseRobot;
    private DistanceSensor m_sensorRange;
    private double m_offset;

    public Sensor_RangeSensor(BaseRobot p_baseRobot, final String p_distanceSensor
        , double p_offset) {
        m_baseRobot = p_baseRobot;
        m_sensorRange = m_baseRobot.hardwareMap.get(DistanceSensor.class, "frontdistance");
        m_offset = p_offset;
    }

    public double getDistance() {
        return m_sensorRange.getDistance(DistanceUnit.INCH) +m_offset;
    }
}
