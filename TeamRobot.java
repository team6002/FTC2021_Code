package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import com.qualcomm.robotcore.hardware.LED;

public class TeamRobot extends BaseRobot {
    SUB_FiniteStateMachine m_FiniteStateMachine;
    FSM_TeamElementArm m_FSM_TEA;
    SUB_Intake m_Intake;
    SUB_Elevator m_Elevator;
    SUB_Bucket m_Bucket;
    SUB_Carousel m_Carousel;
    SUB_PixyCam m_PixyCam; // port 0 = pixydigital, port 1 = pixyanalog
    //SUB_HubDistanceSensor m_HubDistanceSensor;
    SUB_OdomLift m_leftOdomLift;
    SUB_OdomLift m_rightOdomLift;
    SUB_Sensor_IRBucket m_sensorIRBucket;
    Sensor_RangeSensor m_FrontRangeSensor;
    SUB_TEA_elbow m_TEA_elbow;
    SUB_TEA_wrist m_TEA_wrist;
    SUB_TEA_claw m_TEA_claw;



    SUB_OpenCVBase m_BackCamera;
    SUB_OpenCvAlongVuforia m_FrontCamera;

    // REV throughbore encoders.
    private final double m_ticksPerRev = 8192.0;


    @Override
    public void initialize() {
        // 16646 Bot:
        // m_wheelDiameter = 1.496;
        m_trackWidth = 11.2;
        m_centerWheelOffset = -5.1;

        super.initialize();

        m_Drivetrain.invertDriveMotors(true, true, true, true);
        m_Drivetrain.invertStrafeDirection(false);
        m_Drivetrain.invertHeading(true);
        m_Odometry.invertEncoders(false, false, false);


        // Camera is 7" in front of center (-7), 1" left of center (11), and 3" off
        // ground (3)
        float cameraPosFrontOfCenter = (float) -7.5;
        float cameraPosLeftOfCenter = (float) 2.5;
        float cameraPosOffGround = (float) 4.5;
        
        // m_VuforiaNav = new VuforiaNav(this, cameraPosFrontOfCenter, cameraPosLeftOfCenter, cameraPosOffGround);
        // m_VuforiaNav.activate();
        
        m_FrontCamera = new SUB_OpenCvAlongVuforia(this, "Webcam 1", cameraPosFrontOfCenter, cameraPosLeftOfCenter, cameraPosOffGround);
        m_BackCamera = new SUB_OpenCVBase(this, "Webcam 2");

        m_FiniteStateMachine = new SUB_FiniteStateMachine(this);
        m_FSM_TEA = new FSM_TeamElementArm(this);
        
        m_Intake = new SUB_Intake(this, "intakemotor");
        m_Elevator = new SUB_Elevator(this, "elevatormotor");
        m_Bucket = new SUB_Bucket(this,"bucketservo");
        m_Carousel = new SUB_Carousel(this, "carouselmotor");
        m_PixyCam = new SUB_PixyCam(this, "pixydigital", "pixyanalog");
        //m_HubDistanceSensor = new SUB_HubDistanceSensor(this, "HubDistSensor");
        m_leftOdomLift = new SUB_OdomLift(this,"leftodomlift",false,0);
        m_rightOdomLift = new SUB_OdomLift(this,"rightodomlift",true,0);
        m_sensorIRBucket = new SUB_Sensor_IRBucket(this,"irbucket");
        m_FrontRangeSensor = new Sensor_RangeSensor(this,"frontdistance",8.50);
        
        // m_Arm = new SUB_Arm(this, "clawservo", "wristservo", "armmotor");
        m_TEA_elbow = new SUB_TEA_elbow(this,"armmotor");
        m_TEA_wrist = new SUB_TEA_wrist(this,"wristservo");
        m_TEA_claw = new SUB_TEA_claw(this,"clawservo");
        
    }
}

