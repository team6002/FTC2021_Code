package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


// @Autonomous(name="Blue Warehouse", group="Auto Blue")
@Autonomous(name="Blue Warehouse", group="Auto Blue", preselectTeleOp="Mecanum_Blue")
public class AUTO_Blue_Warehouse extends AUTO_Red_Warehouse{

    @Override
    public void initialize() {
        setBlueSide(); 
        super.initialize();
    }

}
