package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;


/**
 * A command to drive the robot with joystick input.
 */
public class CMD_IntakeDefault extends CommandBase {
    private final SUB_Intake m_Intake;
    private final GamepadEx m_driverOP;


    public CMD_IntakeDefault(SUB_Intake p_Intake, GamepadEx driverOp) {

        m_Intake = p_Intake;
        m_driverOP = driverOp; // gamepad of driver

        addRequirements(m_Intake);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
        double leftTriggerLevel = m_driverOP.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        
        if (leftTriggerLevel > .25 ) // trigger is pulled 
        {
            if (!m_Intake.getReverseTriggered()) {
                m_Intake.setReverseTriggered(leftTriggerLevel);
            }

        } else 
        { // not currently triggered
            if (m_Intake.getReverseTriggered()) // was it triggered recently?
            { 
                m_Intake.setReverseTriggeredReset();
            }    
        }
    }

}



