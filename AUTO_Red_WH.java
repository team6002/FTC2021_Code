package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.commands.button.*;
import org.firstinspires.ftc.teamcode.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.ftclib.command.ParallelCommandGroup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// @Autonomous(name="Red_Warehouse", group="Auto Red")
// @Autonomous(name="Red WH RangeSensor", group="Auto Red", preselectTeleOp="Mecanum_Red")
public class AUTO_Red_WH extends TeamRobot {

    int m_Level;

    @Override
    public void initialize() {
        super.initialize();

        m_Odometry.updatePose(82, redSide(-62), Math.toRadians( redSide(-90) ));
        
        // set the correct carousel rotation for red or blue side
        if ( getRedSide() ) m_Carousel.setRedAlliance();
            else m_Carousel.setBlueAlliance();

        m_leftOdomLift.setStateHome();
        m_rightOdomLift.setStateHome();
        
    }

    @Override
    public void endOfOpMode() {
    }

    @Override
    public void waitForStartPeriodic() {
        super.waitForStartPeriodic();

        m_PixyCam.telemetry(); // display telemetry
        m_Odometry.telemetry();
    }

    @Override
    public void afterWaitForStart() {
        m_Level = m_PixyCam.getVotedLevelCount();
        // m_VuforiaNav.deactivateTensorflow();
//      // For some unknown reason, 16646's robot remembers the last run's
//      // location. So another resetEncoder call is needed here.
        // m_Odometry.resetEncoder();

        SequentialCommandGroup m_Commands = new SequentialCommandGroup(
            commandRunAuto()
            );
         // schedule the command
         schedule(m_Commands);
    }

    private SequentialCommandGroup commandRunAuto() {
        SequentialCommandGroup cmds = new SequentialCommandGroup();

        // move elevator to correct level based on bar code
        if (m_Level == 1) {
            cmds.addCommands(new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        } 
        else if (m_Level == 2) {
            cmds.addCommands(new CMD_SetStateLevel2(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        } else {
            // default level
            cmds.addCommands(new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        }

        // drive to shipping hub
        cmds.addCommands(new DriveGotoPosition(this, 71.0, redSide(-41.5)).angle(redSide(-50))
            .speed(.6).posBufferX(1).posBufferY(1));

        cmds.addCommands(new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
        // cmds.addCommands(new OdometryUpdByVuforia(m_Odometry, m_VuforiaNav));


        for (int i=0; i<2;i++) {
            // update odometry with vuforia
            // cmds.addCommands(new DriveGotoPosition(this, 74, redSide(-52))
            //     .speed(.7).posBufferX(4).posBufferY(4));      
            // cmds.addCommands(new Sleep(300));
            // cmds.addCommands(new OdometryUpdByVuforia(m_Odometry, m_VuforiaNav));

            // drive to warehouse front
            cmds.addCommands(new DriveGotoPosition(this, 83, redSide(-60)).speed(.7));
            // cmds.addCommands(new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
            cmds.addCommands( new DriveTurnToAngle(this, redSide(-5)).speed(.5).angleBuffer(1));
            cmds.addCommands(new CMD_DriveMoveTime(m_Drivetrain, 300)
                    .strafe(redSide(.35)).speed(0).turn(0) );
            
            // drive into warehouse and turn on intake 
            cmds.addCommands(new ParallelCommandGroup( 
                new DriveGotoPosition(this, 118, redSide(-62)).angle(redSide(-5))
                    .speed(.8).posBufferX(4).posBufferY(4)
                , new SequentialCommandGroup(
                    new Sleep(300)
                    , new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    )
                )
            );
                
                
            // turn toward the wall
            cmds.addCommands( new DriveTurnToAngle(this, redSide(0)).speed(.6));
            // drive forward, stop when a freight is in bucket
            cmds.addCommands(new CMD_DriveIntake(m_Drivetrain,m_Intake, m_sensorIRBucket, 3000)
                    .strafe( redSide(-.15) ).speed(.20).turn(0) );
            cmds.addCommands(new CMD_IntakeSetOff(m_Intake));
                    
            // move close to the exit
            
            cmds.addCommands(new ParallelCommandGroup( 
                    new CMD_BucketSetHome(m_Bucket)
                    , new DriveGotoPosition(this, 106, redSide(-62)).speed(1).angle(redSide(0))
                       .speed(.8).posBufferX(4).posBufferY(4))
                    
                );
            
            // elevator to level 3
            cmds.addCommands(new CMD_IntakeOnForward(m_Intake));
            cmds.addCommands(new CMD_SetStateLevel3(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
    
            
            cmds.addCommands(new CMD_IntakeSetReverse(m_Intake));
            
            // move sideway toward the wall
            cmds.addCommands( new DriveTurnToAngle(this, redSide(0)).speed(.6).angleBuffer(1));
            cmds.addCommands(new CMD_DriveMoveTime(m_Drivetrain, 350)
                    .strafe(redSide(.35)).speed(0).turn(redSide(0)) );
            cmds.addCommands(new Sleep(200));
            
            cmds.addCommands(new ParallelCommandGroup( 
                new CMD_DriveMoveTime(m_Drivetrain, 100).strafe(redSide(.35)).speed(0).turn(0)
                , new CMD_OdometryUpd_X_byDistanceSensor(m_Odometry, m_FrontRangeSensor,redSide(-64.0),0.0)
                ));

            // drive out the warehouse)
            cmds.addCommands(new DriveGotoPosition(this, 88, redSide(-64)).speed(1).angle(redSide(0))
                .speed(.8).posBufferX(4).posBufferY(4));
                
            // drive to shipping hub
            cmds.addCommands(new DriveGotoPosition(this, redSide(70,75), redSide(-39,38)).angle(redSide(-60))
                .speed(.7).posBufferX(1).posBufferY(1));
            cmds.addCommands( new DriveTurnToAngle(this, redSide(-50)).speed(.6));

            cmds.addCommands(new CMD_SetStateScore(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
            
        };

            // drive to warehouse front
            cmds.addCommands(new DriveGotoPosition(this, 83, redSide(-60)).speed(.7));
            // cmds.addCommands(new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
            cmds.addCommands( new DriveTurnToAngle(this, redSide(-5)).speed(.5).angleBuffer(1));
            cmds.addCommands(new CMD_DriveMoveTime(m_Drivetrain, 300)
                    .strafe(redSide(.35)).speed(0).turn(0) );
            
            // drive into warehouse 
            cmds.addCommands(new ParallelCommandGroup( 
                new DriveGotoPosition(this, 118, redSide(-62)).angle(redSide(-5))
                    .speed(.8).posBufferX(4).posBufferY(4)
                , new SequentialCommandGroup(
                    new Sleep(300)
                    , new CMD_SetStateReadyToIntake(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket)
                    )
                )
            );
            
            // turn toward the wall
            cmds.addCommands( new DriveTurnToAngle(this, redSide(0)).speed(.6));

            // drive forward, stop when a freight is in bucket
            cmds.addCommands(new CMD_DriveIntake(m_Drivetrain,m_Intake, m_sensorIRBucket, 3000)
                    .strafe( redSide(-.15) ).speed(.20).turn(0) );
            cmds.addCommands(new CMD_IntakeSetOff(m_Intake));

            // move close to the exit
            cmds.addCommands(new DriveGotoPosition(this, 112, redSide(-62)).speed(1).angle(redSide(0))
            .speed(.8).posBufferX(4).posBufferY(4));
            
            // elevator to level 3
            cmds.addCommands(new CMD_IntakeOnForward(m_Intake));
            
            cmds.addCommands(new CMD_SetStateLevel1(m_FiniteStateMachine,m_Intake,m_Elevator,m_Bucket));
            // move sideway toward the wall
                
                

        return cmds;
    }

}

