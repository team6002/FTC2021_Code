package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


// @Autonomous(name="Blue Carousel", group="Auto Blue")
@Autonomous(name="Blue Carousel", group="Auto Blue", preselectTeleOp="Mecanum_Blue")
public class AUTO_Blue_Carousel extends AUTO_Red_Carousel{

    @Override
    public void initialize() {
        setBlueSide(); 
        super.initialize();
    }

}
