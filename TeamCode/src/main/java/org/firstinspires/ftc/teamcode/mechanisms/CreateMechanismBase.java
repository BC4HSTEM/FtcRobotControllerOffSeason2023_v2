package org.firstinspires.ftc.teamcode.mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.sleevereader.subsystems.SleeveSubsystem;

public abstract class CreateMechanismBase implements Mechanism{
    protected String m_name = this.getClass().getSimpleName();

    protected SubsystemBase mechanismSubsystem;

    protected HardwareMap hwMap;
    protected String deviceName;
    protected Telemetry telemetry;
    protected GamepadEx op;

    public CreateMechanismBase(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        this.op = op;
        CreateMechanismBaseBase(hwMap, deviceName, telemetry);

    }

    public CreateMechanismBase( HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        this.op = op;
        CreateMechanismBaseBase(hwMap, deviceName, telemetry);

        if (autoCreate) create();

    }

    public CreateMechanismBase(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        CreateMechanismBaseBase(hwMap, deviceName, telemetry);

    }

    private void CreateMechanismBaseBase(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;

        this.telemetry = telemetry;
    }

    public String getName() {
        return this.getClass().getSimpleName();
    }

    public void setName(String name) {
        m_name = name;
    }


}
