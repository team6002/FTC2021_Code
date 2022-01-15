package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.TelemetryEx;
import org.firstinspires.ftc.teamcode.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftclib.command.button.Button;
import org.firstinspires.ftc.teamcode.ftclib.command.button.GamepadButton;
import org.firstinspires.ftc.teamcode.ftclib.command.button.Trigger;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.ftclib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.ftclib.command.Command;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.button.TRG_Subsystem;


public class BaseRobot extends CommandOpMode {

    public int m_RedMultiplier = 1; // red =1, blue = -1

    public TelemetryEx telemetry = TelemetryEx.getInstance();

    public Drivetrain m_Drivetrain;
    public Odometry m_Odometry;
    VuforiaNav m_VuforiaNav;
    public GlobalVariables m_globalVariables = GlobalVariables.getInstance();

    // Robotshop 38mm omni wheel
    double m_wheelDiameter = 1.496;

    // REV throughbore encoders.
    double m_ticksPerRev = 8192.0;

    // Gobida chassis standard configuration
    double m_trackWidth = 15.3065;
    double m_centerWheelOffset = -5.84;

    @Override
    public void initialize() {
        
        reset();

        // Pass in the regular OpMode.telemetry.  Second parm of true enables
        // web-based logging.  Then use telemetryEx.addData() for all logging:
        //  telemetryEx.addData("caption", "value");
        //  telemetryEx.addData("caption", "value %.2f %s %d", 1.23, "abc", 4);
        telemetry.init(super.telemetry, true);

        m_Drivetrain = new Drivetrain(this, "leftfront", "rightfront", "leftback", "rightback");
        m_Odometry = new Odometry(this, "leftfront", "rightfront", "rightback",
                                      m_trackWidth, m_wheelDiameter, m_centerWheelOffset,
                                      m_ticksPerRev);
    }

    public double setSideMultiplier(double value) {
        return (m_RedMultiplier * value);
    }
    
    public boolean getRedSide() {
        return (m_RedMultiplier == 1);
    }
    
    public void setRedSide(){
        m_RedMultiplier = 1;
    }

    public void setBlueSide() {
        m_RedMultiplier = -1;
    }

    public double redSide(double value) {
        return m_RedMultiplier == 1 ? value : -value;
    }

    public double redSide(double redValue, double blueValue) {
        return m_RedMultiplier == 1 ? redValue : blueValue;
    }

    public double blueSide(double value) {
        return m_RedMultiplier == 1 ? -value : value;
    }

    public double blueSide(double blueValue, double redValue) {
        return m_RedMultiplier == 1 ? redValue : blueValue;
    }

    public Button AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
        return (new GamepadButton(gamepad, button)).whenPressed(command);
    }

    public void AddButtonCommand(GamepadEx gamepad, GamepadKeys.Button button
        , TriggerSubsystemBase m_FiniteStateMachine, String m_robotState
        , Command command) {
        
        (new GamepadButton(gamepad, button))
            .and( new TRG_Subsystem(m_FiniteStateMachine, m_robotState))
            .whenActive(command);
    }

    public Button AddButtonCommandHeld(GamepadEx gamepad, GamepadKeys.Button button, Command command) {
        return (new GamepadButton(gamepad, button)).whileHeld(command);
    }

}
