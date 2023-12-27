package org.firstinspires.ftc.teamcode.mechanisms.drone_launcher;

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
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberRightCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.subsystems.PixelGrabberSubsystem;

import java.util.function.DoubleSupplier;

public class CreateDroneLauncherMechanism extends CreateMechanismBase {

    private DroneLauncherSubsystem droneLauncherSubsystem;
    private ServoEx droneLauncher;

    private DroneLauncherLaunchCommand droneLauncherLaunchCommand;

    private int MIN_ANGLE = 0;
    private int MAX_ANGLE = 180;

    private DoubleSupplier rtDS;
    private DoubleSupplier ltDS;
    public CreateDroneLauncherMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);

    }

    //24. Constructor for Teleop .... notice the GamepdEx variable
    public CreateDroneLauncherMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);

    }

    //25. Constructor for Autonomous .... notice there is no GamePadEx variable
    public CreateDroneLauncherMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){
        //26. Create the subsystem and commands
        createBase();

        rtDS = (() -> op.getTrigger(RIGHT_TRIGGER));
        ltDS = (() -> op.getTrigger(LEFT_TRIGGER));

        Trigger rTrigger = new Trigger(() -> op.getTrigger(RIGHT_TRIGGER) > 0.5);
        Trigger lTrigger = new Trigger(() -> op.getTrigger(LEFT_TRIGGER) > 0.5);


        rTrigger.and(lTrigger.whileActiveOnce(droneLauncherLaunchCommand));
        ;

    }

    @Override
    public void createBase(){
        //27. get the servo from the hardware
        droneLauncher = new SimpleServo(hwMap, deviceName, MIN_ANGLE, MAX_ANGLE);

        //28. create the subsystem
        droneLauncherSubsystem = new DroneLauncherSubsystem(droneLauncher, telemetry, true);

        //29. Create the commands, used functions so that autonomous would have less work to do when
        //creating the commands for that opmode
        droneLauncherLaunchCommand = createDroneLauncherLaunchCommand();

    }

    public DroneLauncherLaunchCommand createDroneLauncherLaunchCommand(){

        return new DroneLauncherLaunchCommand(droneLauncherSubsystem, telemetry);
    }
}
