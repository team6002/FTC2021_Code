package org.firstinspires.ftc.teamcode.ftclib.controller;

import org.firstinspires.ftc.teamcode.ftclib.hardware.motors.Motor;
// import com.arcrobotics.ftclib.hardware.motors.Motor;

public class PController extends PDController {

    /**
     * Default constructor, only takes a p-value.
     *
     * @param kp The value of kP for the coefficients.
     */
    public PController(double kp) {
        super(kp, 0);
    }

    /**
     * The extended constructor.
     */
    public PController(double kp, double sp, double pv) {
        super(kp, 0, sp, pv);
    }

}
