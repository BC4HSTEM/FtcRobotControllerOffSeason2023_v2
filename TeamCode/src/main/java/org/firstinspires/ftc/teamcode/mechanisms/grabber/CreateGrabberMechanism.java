package org.firstinspires.ftc.teamcode.mechanisms.grabber;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.commands.GrabberCloseCommand;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.commands.GrabberCommand;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.subsystems.GrabberSubsystem;

public class CreateGrabberMechanism extends CreateMechanismBase {
    private GrabberSubsystem grabberSubsystem;
    private ServoEx gr;

    private GrabberCommand grabberCommand;
    private GrabberCloseCommand grabberCloseCommand;

    private int MIN_ANGLE = 0;
    private int MAX_ANGLE = 180;

    public CreateGrabberMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);

    }

    public CreateGrabberMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);

    }

    public CreateGrabberMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){

        createBase();

        //How to Implement a Toggle with a Button Instead:
        op.getGamepadButton(GamepadKeys.Button.A).whenReleased(grabberCloseCommand);
        op.getGamepadButton(GamepadKeys.Button.A).whenHeld(grabberCommand);

    }

    @Override
    public void createBase(){
        gr = new SimpleServo(hwMap, deviceName, MIN_ANGLE, MAX_ANGLE);
        grabberSubsystem = new GrabberSubsystem(gr);

        grabberCommand = createGrabberCommand();
        grabberCloseCommand = createGrabberCloseCommand();

    }

    public GrabberCommand createGrabberCommand(){

        return new GrabberCommand(grabberSubsystem, telemetry);
    }

    public GrabberCommand getGrabberCommand (){

        return grabberCommand;
    }

    public GrabberCloseCommand createGrabberCloseCommand(){
        return new GrabberCloseCommand(grabberSubsystem, telemetry);
    }

    public GrabberCloseCommand getGrabberCloseCommand(){
        return grabberCloseCommand;
    }


}