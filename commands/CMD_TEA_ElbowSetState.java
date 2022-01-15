package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;

public class CMD_TEA_ElbowSetState extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SUB_TEA_elbow m_elbow;
    private final int m_wantedState;
    private boolean m_waitForCompletion = true;


    // @param subsystem The subsystem used by this command.
    public CMD_TEA_ElbowSetState(SUB_TEA_elbow subsystem, int p_wantedState) {
        m_elbow = subsystem;
        m_wantedState = p_wantedState;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_elbow.setWantedState(m_wantedState);
    }

    @Override
    public boolean isFinished() {
        return ((m_elbow.getState() == m_wantedState));
    }
}
