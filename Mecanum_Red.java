package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.commands.button.*;

import org.firstinspires.ftc.teamcode.ftclib.command.button.Button;
import org.firstinspires.ftc.teamcode.ftclib.command.button.Trigger;
import org.firstinspires.ftc.teamcode.ftclib.command.button.GamepadButton;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.ftclib.command.*;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
// @Disabled
public class Mecanum_Red extends TeamRobot {

    private GamepadEx m_driverOp;
    // private GamepadEx m_toolOp;

    @Override
    
    public void initialize() {
        super.initialize();
        
        m_Drivetrain.m_modeFieldCentric = false;
        m_Odometry.updatePose(m_globalVariables.getRobotPose());

        // m_Odometry.updatePose(0, redSide(0), Math.toRadians(0));
        // m_Odometry.updatePose(34, redSide(-62), Math.toRadians(-90));

        // m_leftOdomLift.setStateRaised();
        // m_rightOdomLift.setStateRaised();

        m_leftOdomLift.setStateHome();
        m_rightOdomLift.setStateHome();
        
        m_driverOp = new GamepadEx(gamepad1);
        // m_toolOp = new GamepadEx(gamepad2);

        // set the correct carousel rotation for red or blue side
        if ( getRedSide() ) m_Carousel.setRedAlliance();
            else m_Carousel.setBlueAlliance();
         
        m_Drivetrain.setDefaultCommand(new DriveDefault(m_Drivetrain, m_Odometry, m_driverOp, 0, 0.02));
        m_Intake.setDefaultCommand(new CMD_IntakeDefault(m_Intake, m_driverOp));
        
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON, new CMD_Drivetrain_SetControlToggle(m_Drivetrain));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON, new Cancel(m_Drivetrain));

        // overload "exception" Team Element Arm
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP, m_FSM_TEA, "HOME"
            , new SequentialCommandGroup
            (   // breaking the move into two segement so we can control the power of each segment
                new CMD_TEA_SetStateReadyToScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
                , new CMD_TEA_SetStateReadyToPickUp(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
            )
        );
        
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP, m_FSM_TEA, "READY2SCORE"
            , new CMD_TEA_SetStateHome(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw)); // 3 = ready to score state
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP, m_FSM_TEA, "SCORE"
            , new CMD_TEA_SetStateReadyToScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw)); // 3 = ready to score state
        // AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP, m_FSM_TEA, "SCORE_RELEASE"
        //     , new SequentialCommandGroup
        //     (   // breaking the move into two segement so we can control the power of each segment
        //         new CMD_TEA_SetStateReadyToScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
        //         , new CMD_TEA_SetStateHome(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
        //     )
        // );
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP, m_FSM_TEA, "READY2PICKUP"
            , new SequentialCommandGroup
            (   // breaking the move into two segement so we can control the power of each segment
                new CMD_TEA_SetStateReadyToScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
                , new CMD_TEA_SetStateHome(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
            )
        );
        // overload "exception" Team Element Arm



        // overload "next" Team Element Arm

        AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, m_FSM_TEA, "HOME"
            , new CMD_TEA_SetStateReadyToScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw)); // 3 = ready to score state

        AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, m_FSM_TEA, "READY2SCORE"
            , new CMD_TEA_SetStateScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw)); // 3 = ready to score state

        AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, m_FSM_TEA, "SCORE"
            , new SequentialCommandGroup
            (   // breaking the move into two segement so we can control the power of each segment
                new CMD_TEA_SetStateScoreRelease(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
                , new Sleep(300)
                , new CMD_TEA_SetStateHome(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw) // 3 = ready to score state
            )

        ); // 3 = ready to score state

        // AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, m_FSM_TEA, "SCORE_RELEASE"
        //     , new CMD_TEA_SetStateReadyToPickUp(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw)); // 3 = ready to score state

        AddButtonCommand(m_driverOp, GamepadKeys.Button.LEFT_BUMPER, m_FSM_TEA, "READY2PICKUP"
            , new SequentialCommandGroup  
                (
                    new CMD_TEA_ClawSetClose(m_TEA_claw)
                    , new Sleep(200)
                    , new CMD_TEA_SetStateReadyToScore(m_FSM_TEA,m_TEA_elbow, m_TEA_wrist, m_TEA_claw)
                )
        ); 

        // overload "next" Team Element Arm


        // overload "next" right Bumper
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, m_FiniteStateMachine, "HOME"
            , new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, m_FiniteStateMachine, "CAPTURE"
            , new SequentialCommandGroup 
                (
                    new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    , new Sleep(300)
                    , new CMD_SetStateHome(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                )
        );
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, m_FiniteStateMachine, "LEVEL1"
            , new SequentialCommandGroup 
                (
                    new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    , new Sleep(300)
                    , new CMD_SetStateHome(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                )
        );
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, m_FiniteStateMachine, "LEVEL2"
            , new SequentialCommandGroup 
                (
                    new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    , new Sleep(300)
                    , new CMD_SetStateHome(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                )
        );
        AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, m_FiniteStateMachine, "LEVEL3"
            , new SequentialCommandGroup 
                (
                    new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    , new Sleep(300)
                    , new CMD_SetStateHome(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                )
        );
        // AddButtonCommand(m_driverOp, GamepadKeys.Button.RIGHT_BUMPER, m_FiniteStateMachine, "INTAKE"
        //     , new CMD_SetStateLevelLastScored(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        // overload right Bumper

        // overload Y button
        AddButtonCommand(m_driverOp, GamepadKeys.Button.Y, m_FiniteStateMachine, "HOME"
            , new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.Y, m_FiniteStateMachine, "LEVEL1"
            , new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.Y, m_FiniteStateMachine, "LEVEL2"
            , new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.Y, m_FiniteStateMachine, "LEVEL3"
            , new CMD_SetStateLevel2(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        // overload Y button

        // overload X button
        AddButtonCommand(m_driverOp, GamepadKeys.Button.X, m_FiniteStateMachine, "INTAKE"
            , new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.X, m_FiniteStateMachine, "HOME"
            , new CMD_SetStateLevel2(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.X, m_FiniteStateMachine, "LEVEL1"
            , new CMD_SetStateLevel2(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.X, m_FiniteStateMachine, "LEVEL2"
            , new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.X, m_FiniteStateMachine, "LEVEL3"
            , new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        // overload X button

       // start Auto drive from warehouse to alliance hub
        (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
            .and(new TRG_Sensor(m_sensorIRBucket))
            .whenActive (
                new SequentialCommandGroup(
                    new CMD_DriveMoveTime(m_Drivetrain, 500)
                        .strafe(redSide(.4)).speed(0).turn(redSide(.05)) 
                    , new Sleep(300)
                    , new CMD_OdometryUpd_X_byDistanceSensor(m_Odometry, m_FrontRangeSensor,redSide(-64.0),0.0)
                    , new DriveGotoPosition(this, 88, redSide(-65)).speed(1).angle(redSide(0))
                        .speed(1).posBufferX(4).posBufferY(4).timeout(3000)
                    , new DriveGotoPosition(this, 69.0, redSide(-37.5)).angle(redSide(-60))
                        .speed(1).posBufferX(1).posBufferY(1).timeout(2000)
                )            );
        
        // auto drive back to warehouse from alliance hub
        (new GamepadButton(m_driverOp, GamepadKeys.Button.A))
            .and( new TRG_Subsystem(m_FiniteStateMachine, "HOME"))
            .whenActive 
            (
                new SequentialCommandGroup
                (
                    new DriveGotoPosition(this, 87, redSide(-59)).speed(1).timeout(3000)
                    , new DriveTurnToAngle(this, redSide(-8)).speed(.5).angleBuffer(1)
                    // press against wall
                    , new CMD_DriveMoveTime(m_Drivetrain, 300)
                        .strafe(redSide(.35)).speed(0).turn(0)
                    // update odometry
                    , new CMD_Odometry_SetY(m_Odometry,redSide(-64))
                    , new DriveGotoPosition(this, 113, redSide(-64)).angle(redSide(-8))
                                .speed(1).posBufferX(4).posBufferY(4).timeout(3000)
                    // , new ParallelCommandGroup
                    //     ( 
                    //         new DriveGotoPosition(this, 113, redSide(-64)).angle(redSide(-8))
                    //             .speed(1).posBufferX(4).posBufferY(4).timeout(3000)
                    //         , new SequentialCommandGroup
                    //         (
                    //             new Sleep(300)
                    //             , new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    //         )
                    //     )
            )
        );

        // Auto drive from warehouse to share hub
        (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
            .and(new TRG_Sensor(m_sensorIRBucket))
            .whenActive (
                new SequentialCommandGroup(
                    new CMD_DriveMoveTime(m_Drivetrain, 600)
                        .strafe(redSide(-.6)).speed(0).turn(redSide(.00)) 
                    , new Sleep(300)
                    , new CMD_OdometryUpd_Y_byDistanceSensor(m_Odometry, m_FrontRangeSensor,redSide(-1),136,redSide(-90))
                    , new DriveGotoPosition(this, 136, redSide(-18)).speed(1).angle(redSide(-95))
                        .speed(1).posBufferX(4).posBufferY(4).timeout(1000)
                    , new DriveGotoPosition(this, 129, redSide(-15)).speed(1).angle(redSide(-45,55))
                        .speed(1).posBufferX(2).posBufferY(2).timeout(1000).angleBuffer(5)
                    , new DriveTurnToAngle(this, redSide(-45,55))
                )            
        );
        
        // auto drive back to warehouse from share hub
        (new GamepadButton(m_driverOp, GamepadKeys.Button.B))
            .and( new TRG_Subsystem(m_FiniteStateMachine, "HOME"))
            .whenActive 
            (
                new SequentialCommandGroup
                (
                    new DriveGotoPosition(this, 133, redSide(-16)).speed(1).timeout(2000)
                    , new DriveTurnToAngle(this, redSide(-90)).speed(.5).angleBuffer(3)
                    // press against wall
                    , new CMD_DriveMoveTime(m_Drivetrain, 300)
                        .strafe(redSide(-.45)).speed(0).turn(0)
                    // update odometry
                    , new CMD_Odometry_SetX(m_Odometry,136)
                    , new DriveGotoPosition(this, 136, redSide(-48)).angle(redSide(-90))
                                .speed(1).posBufferX(4).posBufferY(4).timeout(2000)
                    // , new ParallelCommandGroup
                    //     ( 
                    //         new DriveGotoPosition(this, 136, redSide(-48)).angle(redSide(-90))
                    //             .speed(1).posBufferX(4).posBufferY(4).timeout(2000)
                    //         , new SequentialCommandGroup
                    //         (
                    //             new Sleep(100)
                    //             , new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    //         )
                    //     )
            )
        );


        // AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_UP, new CMD_SetStateCapture(m_Intake,m_Elevator,m_Bucket));
        // AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_DOWN, new CMD_CarouselToggleForwardOff(m_Carousel));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_DOWN,
            new ParallelRaceGroup(
                new CMD_DriveMoveTime(m_Drivetrain, 5000).speed(0.05)
                , new CMD_CarouselSpeedUpSlow(m_Carousel)
                )
            );
        

        // AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_RIGHT, 
        //     new SequentialCommandGroup(
        //     )
        // );
        
        AddButtonCommand(m_driverOp, GamepadKeys.Button.DPAD_LEFT, 
            new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));

        AddButtonCommand(m_driverOp, GamepadKeys.Button.START, new CMD_OdomLift_Toggle(m_leftOdomLift,m_rightOdomLift));
        AddButtonCommand(m_driverOp, GamepadKeys.Button.BACK, new Odometry_UpdByOpenCvVuforia(m_Odometry, m_FrontCamera));

        (new TRG_Sensor(m_sensorIRBucket))
            .and( new TRG_Subsystem(m_FiniteStateMachine, "INTAKE"))
            .whenActive (
                new SequentialCommandGroup(
                // reverse intake to remove extra freight
                new CMD_IntakeSetOff(m_Intake)
                , new CMD_BucketSetCheckDuck(m_Bucket)
                , new Sleep(100)
                , new CMD_BucketSetHome(m_Bucket)
                // , new CMD_IntakeSetReverse(m_Intake)
                // , new Sleep(200)
                , new CMD_IntakeOnForward(m_Intake)
                , new CMD_SetStateLevelLastScored(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                , new CMD_IntakeSetReverse(m_Intake)
                )
            );
    }

    // @Override
    // public void waitForStartPeriodic() {
    //     super.waitForStartPeriodic();

    // }

    @Override
    public void afterWaitForStart() {

    }
}
