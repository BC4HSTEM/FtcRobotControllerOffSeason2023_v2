package org.firstinspires.ftc.teamcode.mechanisms.lift;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class CreateLiftMechanism extends CreateMechanismBase {
    private LiftSubsystem liftSubsystem;
    private DcMotor LM;
    private DoubleSupplier liftDownDS;
    private DoubleSupplier liftUpDS;

    public CreateLiftMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);

    }

    public CreateLiftMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);


    }

    public CreateLiftMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){
        liftDownDS = (() -> op.getTrigger(LEFT_TRIGGER));
        liftUpDS = (() -> op.getTrigger(RIGHT_TRIGGER));


        createBase();


    }

    @Override
    public void createBase(){

    }

    public void LiftDownCommmand createLiftDownCommand(){
    }

    public void LiftUpCommand createLiftUpCommand(){
    }

    public LiftSubsystem getLiftSubsystem() {
        return liftSubsystem;
    }
}
