package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;

public class CMD_ElevatorSaveLastScoredLevel extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SUB_Elevator m_elevator;
    
    // @param subsystem The subsystem used by this command.
    public CMD_ElevatorSaveLastScoredLevel(SUB_Elevator subsystem) {
        m_elevator = subsystem;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elevator.setLastScoreLevel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
