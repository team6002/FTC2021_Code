package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.commands.*;
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
public class Mecanum_Blue extends Mecanum_Red {

    @Override
    public void initialize() {
        setBlueSide(); 
        super.initialize();
    }

}