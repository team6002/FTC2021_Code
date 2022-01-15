package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.ParallelRaceGroup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Carousel", group="Auto Red", preselectTeleOp="Mecanum_Red")
public class AUTO_Red_Carousel extends TeamRobot {

    int m_Level;
    AutoSequencer m_AutoSeq = new AutoSequencer(this);

    @Override
    public void initialize() {
        super.initialize();

        m_Odometry.updatePose(34, redSide(-62), Math.toRadians( redSide(-90) ));
        
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
        m_Odometry.updatePose();
        m_Odometry.telemetry();

        m_BackCamera.telemetry();
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
    public void periodic() 
    {
        
        if (!m_AutoSeq.getIsRunning()) 
        {
            switch (m_AutoSeq.getStepNumber()) {
                case 0 : scheduleStep(1,commandPickUpTeamElement());
                    break;
                case 1 : scheduleStep(2,commandDeliverStoredFreight());
                    break;
                case 2 : scheduleStep(3,commandDropDucks());
                    break;
                case 3 : scheduleStep(4,commandUpdOdometry());
                    break;
                case 4 : scheduleStep(5,commandFindDuck());
                    break;
                case 5 : if (m_sensorIRBucket.getState()) scheduleStep(7,commandDeliverDuck());
                            else scheduleStep(6,commandFindDuck2());
                    break;
                case 6 : if (m_sensorIRBucket.getState()) scheduleStep(7,commandDeliverDuck());
                            else scheduleStep(7,commandNoDuck());
                    break;
                case 7 : scheduleStep(8,commandGetReadyToPark());
                    break;
            }
            
            // Wait for last second to park in warehouse
            if (m_AutoSeq.getRuntime() > 27500) {
                // m_leftOdomLift.setStateRaised();
                // m_rightOdomLift.setStateRaised();
                scheduleStep(7,commandParkInWarehouse());
            }
            
        }
    }


    private SequentialCommandGroup commandPickUpTeamElement() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        // determine correct level based on bar code
        CommandBase cmd_Position; 
        CommandBase cmd_Level; 
        if (m_Level == 1) {
            cmd_Level = new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket);
            cmd_Position = new DriveGotoPosition(this, redSide(30,40), redSide(-44)).angle(redSide(-80,100))
                    .speed(.4).posBufferX(1).posBufferY(1);
        } 
        else if (m_Level == 2) {
            cmd_Level = new CMD_SetStateLevel2(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket);
            cmd_Position = new DriveGotoPosition(this, redSide(36,32), redSide(-44.0)).angle(redSide(-99,81))
                    .speed(.4).posBufferX(1).posBufferY(1);
        } else {
            // default level
            cmd_Level = new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket);
            cmd_Position = new DriveGotoPosition(this, redSide(41,26), redSide(-46.0,44)).angle(redSide(-118,62))
                    .speed(.4).posBufferX(1).posBufferY(1);
        }

        // drive to shipping hub while moving to the correct level

        cmds.addCommands(
            new ParallelCommandGroup( 
                new CMD_TEA_SetStateReadyToPickUp(m_FSM_TEA, m_TEA_elbow, m_TEA_wrist, m_TEA_claw)
                , cmd_Level
            )
        );
        cmds.addCommands(cmd_Position);
        cmds.addCommands(new CMD_TEA_ClawSetClose(m_TEA_claw));
        cmds.addCommands(
            new ParallelCommandGroup
            ( 
                new CMD_TEA_SetStateHome(m_FSM_TEA, m_TEA_elbow, m_TEA_wrist, m_TEA_claw)
                , new SequentialCommandGroup
                    (
                        new DriveTurnToAngle(this, redSide(178)).speed(.5)
                        , new Sleep(200)
                        , new Odometry_UpdByOpenCvVuforia(m_Odometry, m_FrontCamera) 
                    )
            )
        );

        return cmds;
    }

    private SequentialCommandGroup commandDeliverStoredFreight() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

    // deliver pre-load based on bar code

        // drive to shipping hub 
        cmds.addCommands(new DriveGotoPosition(this, redSide(46,46), redSide(-37,39.0)).timeout(1000)
                .angle(redSide(-135,145)).speed(.5).posBufferX(1).posBufferY(1).angleBuffer(2)
        );
        cmds.addCommands(new DriveTurnToAngle(this, redSide(-135,145)).speed(.5));

        cmds.addCommands(new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));

        return cmds;
    }

    
    private SequentialCommandGroup commandDropDucks() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        // drive to carousel

        cmds.addCommands
        (
            new ParallelCommandGroup
            ( 
                new SequentialCommandGroup
                (   
                    new DriveGotoPosition(this, 12, redSide(-58)).speed(.5).timeout(2000)
                    // , new DriveGotoPosition(this, 12, redSide(-59))
                    //     .speed(.5).posBufferX(1).posBufferY(1).timeout(2000)
                    , new DriveTurnToAngle(this, redSide(-135))
                )
                , new CMD_SetStateHome(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
            )
        );

        //turn carousel to deliver duck
        cmds.addCommands(
            new ParallelCommandGroup( 
                new CMD_DriveMoveTime(m_Drivetrain, 1500).speed(0.07)
                , new CMD_CarouselSpinAuto(m_Carousel)
            )
        );
        
        // drive forward slowly

        return cmds;
    }
    
    private SequentialCommandGroup commandUpdOdometry()
    {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        
        cmds.addCommands(new DriveGotoPosition(this, 24, redSide(-48)).angle(redSide(160)).timeout(1000));
        cmds.addCommands(new DriveTurnToAngle(this, redSide(160)).speed(.5));
        cmds.addCommands(new Odometry_UpdByOpenCvVuforia(m_Odometry, m_FrontCamera));
        return cmds;
    }
    
    private SequentialCommandGroup commandFindDuck() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        
        cmds.addCommands(new CMD_SetStateReadyToIntake(m_FiniteStateMachine, m_Intake, m_Elevator, m_Bucket));
        // slow down the intake
        cmds.addCommands(new CMD_IntakeSetSpeed(m_Intake, .3));
        cmds.addCommands(new DriveGotoPosition(this, 12, redSide(-60)).timeout(1000)
            .speed(.5).angle(redSide(-45)));

        cmds.addCommands( new Sleep(500));

        double Y = m_Odometry.getY();
        double heading = m_Odometry.getDegrees();
        
        // find the duck by sweeping back and forth
        // time limit 6 sec
        cmds.addCommands(
            new CMD_DrivePositionIntake(this,m_sensorIRBucket, 48, redSide(-62)).angle(redSide(0,10))
                .speed(.7).posBufferX(1).posBufferY(1).timeout(3000)
            , new CMD_BucketSetCheckDuck(m_Bucket)
        );

        return cmds;
    }    

    private SequentialCommandGroup commandFindDuck2() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        
        cmds.addCommands(
            new CMD_BucketSetIntake(m_Bucket)
            , new CMD_IntakeSetReverse(m_Intake) // help push the bucket down
            , new Sleep(200)
            , new CMD_IntakeOnForward(m_Intake) 
            , new DriveTurnToAngle(this, redSide(-125)).speed(.5)
            , new CMD_DrivePositionIntake(this,m_sensorIRBucket, 12, redSide(-62)).angle(redSide(-125))
                    .speed(.6).timeout(3000)
            , new DriveTurnToAngle(this, redSide(125)).speed(.5)
            , new CMD_BucketSetCheckDuck(m_Bucket)
            , new Sleep(200)
        );

        return cmds;
    }    


    private SequentialCommandGroup commandDeliverDuck() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        // Move to level 3 while updating with Vuforia
        // then move to hub
        cmds.addCommands(
            new ParallelCommandGroup(
                new CMD_SetStateLevel3(m_FiniteStateMachine, m_Intake, m_Elevator, m_Bucket)
                , new SequentialCommandGroup(
                    new DriveGotoPosition(this, 36, redSide(-48)).angle(redSide(170)).timeout(1500)
                    , new DriveTurnToAngle(this, redSide(160)).speed(.5)
                    , new Sleep(500)
                    , new Odometry_UpdByOpenCvVuforia(m_Odometry, m_FrontCamera)
                    , new DriveGotoPosition(this, redSide(49,47), redSide(-36,38)).angle(redSide(-135))
                     .speed(.7).posBufferX(1).posBufferY(1).timeout(2000)
                )
            )
        );
        
        cmds.addCommands(new CMD_SetStateScore(m_FiniteStateMachine, m_Intake, m_Elevator, m_Bucket));
        
        return cmds;
    }
    
    private SequentialCommandGroup commandNoDuck() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        // Move to level 3 while updating with Vuforia
        // then move to hub
        cmds.addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new CMD_BucketSetHome(m_Bucket)
                )
                , new SequentialCommandGroup(
                    new DriveGotoPosition(this, 36, redSide(-48)).angle(redSide(170)).timeout(2000)
                    , new DriveTurnToAngle(this, redSide(160)).speed(.5)
                    , new Sleep(500)
                    , new Odometry_UpdByOpenCvVuforia(m_Odometry, m_FrontCamera)
                )
            )
        );
        
        cmds.addCommands(new DriveGotoPosition(this, 38, redSide(-58)).timeout(1500));

        return cmds;
    }

    private SequentialCommandGroup commandGetReadyToPark() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        
        // Wait by the wall until last few second
        // cmds.addCommands(new DriveGotoPosition(this, 60, redSide(-64))
        //     .speed(.7).posBufferX(4).posBufferY(4));

        cmds.addCommands
        (   
            new ParallelCommandGroup
                (
                    new SequentialCommandGroup
                    (
                        new DriveGotoPosition(this, 44, redSide(-64))
                            .speed(.7).posBufferX(4).posBufferY(4).timeout(1000)
                        , new DriveTurnToAngle(this, redSide(-8)).speed(.6).angleBuffer(1)
                        , new CMD_DriveMoveTime(m_Drivetrain, 800)
                            .speed(0).strafe(redSide(.35))
                    )
                    , new CMD_SetStateHome(m_FiniteStateMachine, m_Intake, m_Elevator, m_Bucket)
                )
        );
        
        // update odometry
        cmds.addCommands(new CMD_Odometry_SetY(m_Odometry,redSide(-64)));
        
        return cmds;
    }
    
    private SequentialCommandGroup commandParkInWarehouse() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();
        
        cmds.addCommands(new DriveGotoPosition(this, 118, redSide(-66))
            .angle(redSide(0,10)).posBufferX(4).posBufferY(2).angleBuffer(2));
            
        return cmds;
    }   
    

    
}

