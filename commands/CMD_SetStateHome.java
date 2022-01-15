package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;


public class CMD_SetStateHome extends SequentialCommandGroup {

    public CMD_SetStateHome(SUB_FiniteStateMachine p_FiniteStateMachine, SUB_Intake p_Intake, SUB_Elevator p_Elevator
            , SUB_Bucket p_Bucket) {

        addCommands (
            new CMD_IntakeSetOff(p_Intake)
            , new CMD_BucketSetCapture(p_Bucket)
            , new CMD_ElevatorSetWantedLevel(p_Elevator,0)
            , new CMD_BucketSetHome(p_Bucket)
            , new CMD_SetState(p_FiniteStateMachine, SUB_FiniteStateMachine.RobotState.HOME)
            );
            
        clearGroupedCommands();
    }
}
