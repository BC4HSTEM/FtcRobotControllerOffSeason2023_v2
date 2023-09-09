package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Mechanism {

    default void CreateMechanismBase(){}



    default void create(){
        createBase();
    }

    default void createAuto(){
        createBase();
    }

    default void createBase(){}
}
