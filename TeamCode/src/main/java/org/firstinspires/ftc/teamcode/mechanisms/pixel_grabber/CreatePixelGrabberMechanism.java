package org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberCloseCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.subsystems.PixelGrabberSubsystem;

//18. extend CreateMechanismBase
public class CreatePixelGrabberMechanism extends CreateMechanismBase {
    //19. Define the subsystem for the mechanism
    private PixelGrabberSubsystem grabberSubsystem;
    //20. Define the Servo coming from the hardware map
    private ServoEx grabberRight;
    private ServoEx grabberLeft;

    //21. Define your Commands
    private PixelGrabberCommand grabberCommand;
    private PixelGrabberCloseCommand grabberCloseCommand;

    //22. Is this a 180 or 360 servo, define your max and min
    private int MIN_ANGLE = 0;
    private int MAX_ANGLE = 270;

    //23. Define your constructors
    public CreatePixelGrabberMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);

    }

    //24. Constructor for Teleop .... notice the GamepdEx variable
    public CreatePixelGrabberMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);

    }

    //25. Constructor for Autonomous .... notice there is no GamePadEx variable
    public CreatePixelGrabberMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){
        //26. Create the subsystem and commands
        createBase();

        //30. determine which button you want to use
        //31. assign the command to te appropriate button action https://docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system/binding-commands-to-triggers
        //How to Implement a Toggle with a Button Instead:
        op.getGamepadButton(GamepadKeys.Button.B).whenReleased(grabberCommand);
        op.getGamepadButton(GamepadKeys.Button.B).whenHeld(grabberCloseCommand);

        //32. go to CommandTeleop

    }

    @Override
    public void createBase(){
        //27. get the servo from the hardware
        grabberRight = new SimpleServo(hwMap, "pixel_grabber_right", MIN_ANGLE, MAX_ANGLE);
        grabberRight = new SimpleServo(hwMap, "pixel_grabber_left", MIN_ANGLE, MAX_ANGLE);
        //28. create the subsystem
        grabberSubsystem = new PixelGrabberSubsystem(grabberRight, grabberLeft, telemetry, true);

        //29. Create the commands, used functions so that autonomous would have less work to do when
        //creating the commands for that opmode
        grabberCommand = createGrabberCommand();
        grabberCloseCommand = createGrabberCloseCommand();

    }

    public PixelGrabberCommand createGrabberCommand(){

        return new PixelGrabberCommand(grabberSubsystem, telemetry);
    }

    public PixelGrabberCommand getGrabberCommand (){

        return grabberCommand;
    }

    public PixelGrabberCloseCommand createGrabberCloseCommand(){
        return new PixelGrabberCloseCommand(grabberSubsystem, telemetry);
    }

    public PixelGrabberCloseCommand getGrabberCloseCommand(){
        return grabberCloseCommand;
    }


}