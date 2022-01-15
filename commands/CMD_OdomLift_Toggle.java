package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SUB_OdomLift;


public class CMD_OdomLift_Toggle extends CommandBase  {
    private final SUB_OdomLift m_odomLeft, m_odomRight;

    public CMD_OdomLift_Toggle(SUB_OdomLift p_odomLeft,SUB_OdomLift p_odomRight) {
        m_odomLeft = p_odomLeft;
        m_odomRight = p_odomRight;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements();
    }

    @Override
    public void initialize() {
        m_odomLeft.toggleLift();
        m_odomRight.toggleLift();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }  
  
}
