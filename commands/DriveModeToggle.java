package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;

public class DriveModeToggle extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_subsystem;
    private boolean m_fieldCentric = true;

    // @param subsystem The subsystem used by this command.
    public DriveModeToggle(Drivetrain subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_fieldCentric = !m_fieldCentric;
        m_subsystem.switchDriveMode(m_fieldCentric);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
