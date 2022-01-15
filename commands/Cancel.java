package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;

public class Cancel extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_drivetrain;

    // @param subsystem The subsystem used by this command.
    public Cancel(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
