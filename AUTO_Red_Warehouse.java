package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.commands.button.*;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.ParallelRaceGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// @Autonomous(name="Red_Warehouse", group="Auto Red")
@Autonomous(name="Red Warehouse", group="Auto Red", preselectTeleOp="Mecanum_Red")
public class AUTO_Red_Warehouse extends TeamRobot {

    int m_Level;
    AutoSequencer m_AutoSeq = new AutoSequencer(this);


    @Override
    public void initialize() {
        super.initialize();

        m_Odometry.updatePose(82, redSide(-62), Math.toRadians( redSide(-90) ));
        
        // set the correct carousel rotation for red or blue side
        if ( getRedSide() ) m_Carousel.setRedAlliance();
            else m_Carousel.setBlueAlliance();

        m_leftOdomLift.setStateHome();
        m_rightOdomLift.setStateHome();
        
        m_Elevator.resetEncoder();
        m_TEA_elbow.resetEncoder();
        
    }

    @Override
    public void endOfOpMode() {
    }

    @Override
    public void waitForStartPeriodic() {
        super.waitForStartPeriodic();

        m_BackCamera.telemetry();
        m_Odometry.telemetry();
        m_Elevator.telemetry();
        m_TEA_elbow.telemetry();
        
    }

    @Override
    public void afterWaitForStart() {

        m_Level = m_BackCamera.getAnalysis();
        m_AutoSeq.reset();

    }

    private void scheduleStep(int p_stepNumber, SequentialCommandGroup p_cmd) {
        m_AutoSeq.setStepNumber(p_stepNumber);
        schedule(new SequentialCommandGroup(p_cmd, new AutoSeq_setIsRunning(m_AutoSeq,false)));
    };
    
    @Override
    public void periodic() {
        
        if (!m_AutoSeq.getIsRunning()) {
            switch (m_AutoSeq.getStepNumber()) {
                case 0 : scheduleStep(1,deliverPreload());
                    break;
                case 1 : scheduleStep(2,driveIntoWarehouse(4));
                    break;
                case 2 : if (m_sensorIRBucket.getState())
                            {
                                scheduleStep(3,deliverFreightToHub());
                            } else scheduleStep(2,retriesGetFreight(4));
                    break;
                case 3 : scheduleStep(4,driveIntoWarehouse(4));
                    break;
                case 4 : 
                        if (m_sensorIRBucket.getState()) 
                        {
                            if (m_AutoSeq.getRuntime() < 22000) {
                        // There is enough time to deliver to hub
                                scheduleStep(5,deliverFreightToHub());
                            } else    
                            {
                                scheduleStep(7,parkInWarehouse()); 
                            };
                        } else 
                        {   
                            if (m_AutoSeq.getRuntime() < 28000) 
                            {
                                scheduleStep(4,retriesGetFreight(4));
                            } else scheduleStep(7,parkInWarehouse());
                        }
                    break;
                case 5 : if (m_AutoSeq.getRuntime() < 26000) {
                            scheduleStep(6,driveIntoWarehouse(2));
                        } else { 
                            // not enough time, just park
                            scheduleStep(7,driveIntoWarehouse(0)); };
                    break;
                case 6 : scheduleStep(7,parkInWarehouse());
                    break;
            }
            
            // Out of time, cancel all and finish up
            // if (m_AutoSeq.getRuntime()>27) {
            //     cancelAll();
            // }
            
        }
    }

    private SequentialCommandGroup deliverPreload() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

        // determine correct level based on bar code
        CommandBase cmd_Level; 
        if (m_Level == 1) {
            cmd_Level = new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket);
        } 
        else if (m_Level == 2) {
            cmd_Level = new CMD_SetStateLevel2(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket);
        } else {
            // default level
            cmd_Level = new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket);
        }

        // drive to shipping hub while moving to the correct level
        cmds.addCommands(
            new ParallelCommandGroup( 
                new DriveGotoPosition(this, redSide(69.0,69.0), redSide(-38,40)).angle(redSide(-50))
                    .speed(.5).posBufferX(1).posBufferY(1).timeout(2500)
                , cmd_Level
            )
        );
        
        cmds.addCommands(new Sleep(300));
        cmds.addCommands(
            new ParallelCommandGroup( 
                new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                , new CMD_DriveMoveTime(m_Drivetrain, 200).strafe(0).speed(.2).turn(0)
            )
        );
        
        // cmds.addCommands(new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));

        return cmds; 
    }

    private SequentialCommandGroup driveIntoWarehouse(int p_tries)
    {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

        cmds.addCommands
        (
            new ParallelCommandGroup
            (
                new SequentialCommandGroup
                (
                    new DriveGotoPosition(this, 74, redSide(-54))
                        .speed(.7).posBufferX(4).posBufferY(4).timeout(2000)      
                    , new Sleep(300)
                    , new Odometry_UpdByOpenCvVuforia(m_Odometry, m_FrontCamera)
                )
                , new CMD_SetStateHome(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
            )
        );

        // drive to warehouse front
        cmds.addCommands(new DriveGotoPosition(this, 89, redSide(-62)).speed(.7).timeout(2000));
        cmds.addCommands( new DriveTurnToAngle(this, redSide(-8)).speed(.5));
            
        // press against wall
        cmds.addCommands(new CMD_DriveMoveTime(m_Drivetrain, 300)
                .strafe(redSide(.35)).speed(0).turn(0) );
        // update odometry
        cmds.addCommands(new CMD_Odometry_SetY(m_Odometry,redSide(-64)));
            
        if (p_tries > 0) { // number of attempt to pickup freight
           // drive into warehouse and turn on intake
            cmds.addCommands
            (
                new ParallelCommandGroup
                ( 
                    new DriveGotoPosition(this, 119, redSide(-64)).angle(redSide(-8))
                        .speed(.7).posBufferX(4).posBufferY(4).timeout(2000)
                    , new SequentialCommandGroup
                    (
                        new Sleep(300)
                        , new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    )
                )
            );
     
            cmds.addCommands(new CMD_DriveIntake(m_Drivetrain, m_Intake,m_sensorIRBucket, p_tries)
                    .strafe(redSide(-.3)).speed(-.4));
    
            cmds.addCommands(new CMD_IntakeSetOff(m_Intake));
            cmds.addCommands(new CMD_BucketSetCheckDuck(m_Bucket));
        } else {
            // just drive in
             cmds.addCommands(new DriveGotoPosition(this, 119, redSide(-64)).angle(redSide(-8))
                .speed(.8).posBufferX(4).posBufferY(4).timeout(2000));
        }

        return cmds; 
    }
    

    private SequentialCommandGroup retriesGetFreight(int p_tries)
    {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

        cmds.addCommands(new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        cmds.addCommands(new DriveGotoPosition(this, 119, redSide(-64)).angle(redSide(-8))
                    .speed(.7).posBufferX(4).posBufferY(4).timeout(2000)
        );
 
        cmds.addCommands(new CMD_DriveIntake(m_Drivetrain, m_Intake,m_sensorIRBucket, p_tries)
                .strafe(redSide(-.3)).speed(-.4));

        cmds.addCommands(new CMD_IntakeSetOff(m_Intake));
        cmds.addCommands(new CMD_BucketSetCheckDuck(m_Bucket));

        return cmds; 
    }

    private SequentialCommandGroup deliverFreightToHub() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

            // drive out of warehouse while moving elevator to level 3
            cmds.addCommands
            (
                new ParallelCommandGroup
                ( 
                    new SequentialCommandGroup
                    (
                        // new CMD_IntakeOnForward(m_Intake)
                        // , new Sleep(200)
                        // ,
                        new CMD_BucketSetHome(m_Bucket)
                        , new CMD_IntakeSetReverse(m_Intake)
                        , new Sleep(300)
                        , new CMD_BucketSetCheckDuck(m_Bucket)
                        , new CMD_IntakeOnForward(m_Intake)
                        , new Sleep(300)
                        , new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                        // , new ParallelRaceGroup
                        //     ( 
                        //         new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                        //         , new Sleep(5000)
                        //     )
                        , new CMD_IntakeSetReverse(m_Intake)
                    )
                    , new SequentialCommandGroup
                    (
                        new DriveGotoPosition(this, 106, redSide(-64)).speed(1).angle(redSide(0))
                            .speed(.8).posBufferX(4).posBufferY(4).timeout(2000)
                        , new DriveTurnToAngle(this, redSide(0)).speed(.6).angleBuffer(2)
                        , new CMD_DriveMoveTime(m_Drivetrain, 400).strafe(redSide(.35)).speed(0).turn(0)

                        // start- update odometry using 2m distance sensor                 
                        ,new ParallelCommandGroup
                        ( 
                            new CMD_DriveMoveTime(m_Drivetrain, 100).strafe(redSide(.35)).speed(0).turn(0)
                            , new CMD_OdometryUpd_X_byDistanceSensor(m_Odometry, m_FrontRangeSensor,redSide(-64.0),0.0)
                        )
                        // end- update odometry using 2m distance sensor                 
                        // update odometry
                        , new CMD_Odometry_SetY(m_Odometry,redSide(-64))
                        
                        , new DriveGotoPosition(this, 88, redSide(-64)).speed(1).angle(redSide(0))
                            .speed(.8).posBufferX(4).posBufferY(4).timeout(2000)
                        // drive to shipping hub
                        , new DriveGotoPosition(this, 69, redSide(-37,38.5)).angle(redSide(-60))
                            .speed(.6).posBufferX(1).posBufferY(1).timeout(2000)
                        , new DriveTurnToAngle(this, redSide(-60)).speed(.5).angleBuffer(1)
                            
                    )
                )
            );
            
        cmds.addCommands(new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        // cmds.addCommands(new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        cmds.addCommands(
            new ParallelCommandGroup( 
                new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                , new CMD_DriveMoveTime(m_Drivetrain, 200).strafe(0).speed(.2).turn(0)
            )
        );

        return cmds;
    }


    private SequentialCommandGroup parkInWarehouse() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

            cmds.addCommands
            (
                new DriveGotoPosition(this, 119, redSide(-39))
                    .angle(redSide(-90)).timeout(2000)
                , new DriveGotoPosition(this, 136, redSide(-39))
                    .angle(redSide(-90)).timeout(2000)
            );

        return cmds;
    }
}

