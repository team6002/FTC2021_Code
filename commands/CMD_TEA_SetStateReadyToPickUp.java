package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;


public class CMD_TEA_SetStateReadyToPickUp extends SequentialCommandGroup {
    private final FSM_TeamElementArm m_FSM;

    public CMD_TEA_SetStateReadyToPickUp(FSM_TeamElementArm p_FSM
        , SUB_TEA_elbow p_elbow, SUB_TEA_wrist p_wrist, SUB_TEA_claw p_claw) 
    {
        m_FSM = p_FSM;
        
        addCommands (
            new ParallelCommandGroup
            (
                new CMD_TEA_ElbowSetReadyToPickUp(p_elbow)
                , new CMD_TEA_WristSetReadyToPickUp(p_wrist)
                , new CMD_TEA_ClawSetOpen(p_claw)
            )
            , new CMD_TEA_SetState(p_FSM, FSM_TeamElementArm.State.READY2PICKUP)
        );
    }
}
