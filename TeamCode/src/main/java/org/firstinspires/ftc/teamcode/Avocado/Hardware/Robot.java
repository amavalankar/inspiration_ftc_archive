package org.firstinspires.ftc.teamcode.Avocado.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Avocado.OpModes.Autonomous.Autonomous_Avocado;


/**
 * This file contains all the methods and hardware for Avocado's robot as of 11/19/18
 */

public class Robot extends Autonomous_Avocado {

    Constants constants;
    private ElapsedTime runtime = new ElapsedTime();
    /* Hardware ------------------------------------------------------------------------------------ */

    // Hardware controlled by Gamepad 1
    public DcMotor topLeftMotor;
    public DcMotor bottomLeftMotor;
    public DcMotor topRightMotor;
    public DcMotor bottomRightMotor;

    // Hardware controlled by Gamepad 2
    public DcMotor hanger;
    public DcMotor tiltMotor;
    public DcMotor extension;

    HardwareMap hwmap = null;

    public void init(HardwareMap ahwmap) {
        // Save reference to Hardware map
        hwmap = ahwmap;

        // Hardware map

        topLeftMotor = hwmap.dcMotor.get("topLeftMotor");
        bottomLeftMotor = hwmap.dcMotor.get("bottomLeftMotor");
        topRightMotor = hwmap.dcMotor.get("topRightMotor");
        bottomRightMotor = hwmap.dcMotor.get("bottomRightMotor");
        hanger = hwmap.dcMotor.get("hanger");


    }


    /* Autonomous Methods ----------------------------------------------------------------------------- */

}



