package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;


public class CMD_SetStateReadyToScore extends SequentialCommandGroup {

    public CMD_SetStateReadyToScore(SUB_FiniteStateMachine p_FiniteStateMachine, SUB_Intake p_Intake, SUB_Elevator p_Elevator
            , SUB_Bucket p_Bucket) {
        
        addCommands (
            
            // , new CMD_SetStateSaveLevel(p_StateMachine, p_Intake, p_Elevator, p_Bucket)
             new CMD_BucketSetReadyToScore(p_Bucket)
            , new CMD_IntakeSetOff(p_Intake)
            , new CMD_SetState(p_FiniteStateMachine, SUB_FiniteStateMachine.RobotState.READY2SCORE)
            );
            
        clearGroupedCommands();
    }
}
