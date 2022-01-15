package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;


public class CMD_SetStateReadyToIntake extends SequentialCommandGroup {

    public CMD_SetStateReadyToIntake(SUB_FiniteStateMachine p_FiniteStateMachine, SUB_Intake p_Intake, SUB_Elevator p_Elevator
            , SUB_Bucket p_Bucket) {

        addCommands (
            
            // , new CMD_BucketSetCapture(p_Bucket)
             new CMD_ElevatorSetWantedLevel(p_Elevator,0)
            , new CMD_BucketSetIntake(p_Bucket)
            , new CMD_IntakeOnForward(p_Intake)
            , new CMD_SetState(p_FiniteStateMachine, SUB_FiniteStateMachine.RobotState.INTAKE)
            );
            
        clearGroupedCommands();
    }
}
