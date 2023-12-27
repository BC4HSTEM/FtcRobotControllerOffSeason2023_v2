package org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.commands.DroneLauncherLaunchCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.subsystems.DroneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.commands.GrabberWristDropCommand;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.commands.GrabberWristPickUpCommand;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.subsystems.GrabberWristSubsystem;

import java.util.function.DoubleSupplier;

public class CreateGrabberWristMechanism extends CreateMechanismBase {

    private GrabberWristSubsystem grabberWristSubsystem;
    private ServoEx grabberWrist;

    private GrabberWristDropCommand grabberWristDropCommand;
    private GrabberWristPickUpCommand grabberWristPickUpCommand;

    private int MIN_ANGLE = 0;
    private int MAX_ANGLE = 180;

    public CreateGrabberWristMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);

    }

    //24. Constructor for Teleop .... notice the GamepdEx variable
    public CreateGrabberWristMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);

    }

    //25. Constructor for Autonomous .... notice there is no GamePadEx variable
    public CreateGrabberWristMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){
        //26. Create the subsystem and commands
        createBase();



        op.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(grabberWristDropCommand);
        op.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(grabberWristPickUpCommand);


    }

    @Override
    public void createBase(){
        //27. get the servo from the hardware
        grabberWrist = new SimpleServo(hwMap, deviceName, MIN_ANGLE, MAX_ANGLE);


        //28. create the subsystem
        grabberWristSubsystem = new GrabberWristSubsystem(grabberWrist, telemetry, true);
        grabberWristSubsystem.setInverted(true);

        //29. Create the commands, used functions so that autonomous would have less work to do when
        //creating the commands for that opmode
        grabberWristDropCommand = createGrabberWristDropCommand();
        grabberWristPickUpCommand = createGrabberWristPickUpCommand();

    }

    public GrabberWristDropCommand createGrabberWristDropCommand(){

        return new GrabberWristDropCommand(grabberWristSubsystem, telemetry);
    }

    public GrabberWristPickUpCommand createGrabberWristPickUpCommand(){

        return new GrabberWristPickUpCommand(grabberWristSubsystem, telemetry);
    }
}
