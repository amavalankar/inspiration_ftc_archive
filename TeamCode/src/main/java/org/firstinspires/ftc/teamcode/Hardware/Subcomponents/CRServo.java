package org.firstinspires.ftc.teamcode.Hardware.Subcomponents;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by adityamavalankar on 11/5/18.
 */

public abstract class CRServo implements com.qualcomm.robotcore.hardware.CRServo {

    private com.qualcomm.robotcore.hardware.CRServo crs;
    public HardwareMap hwmap;

    public void init(String hardwareName, HardwareMap inputMap) {

        hwmap = inputMap;

        crs = hwmap.get(com.qualcomm.robotcore.hardware.CRServo.class, hardwareName);
    }



}

//Eesh isn't part of the cool club!