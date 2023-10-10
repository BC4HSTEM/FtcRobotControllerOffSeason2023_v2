package org.firstinspires.ftc.teamcode.mechanisms.lift;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.lift.commands.LiftDownCommand;
import org.firstinspires.ftc.teamcode.mechanisms.lift.commands.LiftUpCommand;
import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class CreateLiftMechanism extends CreateMechanismBase {
    //26. Tyler already created the variable for the liftSubsystem I just added vars for the commands
    private LiftSubsystem liftSubsystem;
    private LiftUpCommand liftUpCommand;
    private LiftDownCommand liftDownCommand;
    //27. Changed DcMotor to MotorEx to match our subsystem implementation
    private MotorEx LM;
    //28. Created Supplier variables to carry the info from the triggers to the commands without us having to
    //do much work
    private DoubleSupplier liftDownDS;
    private DoubleSupplier liftUpDS;

    //29. Tyler created the constructors based on the CreateMechanismBase
    //what's special about the CreateMechanismBase base class is that this is not a part of FTCLib
    //it's something Alex created to demonstrate to the team how base classes work,
    //hopefully we can dive into it's details and it can be discussed with the judges
    public CreateLiftMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);
    }

    //30. autoCreate triggers a method in the base class to tell the mechanism to go ahead and create the subsystem
    //and map the commands to robot (controller, camera, etc)
    //there are times where you want to delay creating the mechanism until it's actually needed
    //most times we just use the autoCreate option
    public CreateLiftMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);
    }

    public CreateLiftMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    //31. When autoCreate is passed into the constructor, then the base class automatically calls the create method
    //recall the use of the @Override item in the Commands, we are doing something similar in concept here
    @Override
    public void create(){
        //32. you can read the functionality of getTrigger here - https://docs.ftclib.org/ftclib/v/v2.0.0/features/gamepad-extensions
        //So what's up with this weird syntax
        //Well that's a Java Lambda expression - some examples here - http://www.java2s.com/Tutorials/Java/java.util.function/Supplier/index.htm
        //here are some good definitions and examples - https://www.w3schools.com/java/java_lambda.asp
        //lambda functions are anonymous functions that can hold a value and update when the value changes
        //ie when the trigger changes
        //Note, this is the value of the trigger, not the controller itself
        liftDownDS = (() -> op.getTrigger(LEFT_TRIGGER));
        liftUpDS = (() -> op.getTrigger(RIGHT_TRIGGER));

        //33. after setting up our triggers we can now creates our subsystem
        createBase();

        //39. created commands

        //40. set motor to run without encoders
        liftSubsystem.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //41. instead of creating the command right here, we created a function for it so we
        //can reuse the code for Auto
        liftUpCommand = createLiftUpCommand();
        liftDownCommand = createLiftDownCommand();

        //42. create the actual triggers
        //https://docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system/binding-commands-to-triggers
        Trigger lTrigger = new Trigger(() -> op.getTrigger(LEFT_TRIGGER) > 0.5);
        Trigger rTrigger = new Trigger(() -> op.getTrigger(RIGHT_TRIGGER) > 0.5);

        //43. bind the commands to the triggers
        //commands only execute when the trigger is greater than 0.5 pressed
        //these may have to be reversed based on what the team wants
        rTrigger.whenActive(liftUpCommand);
        lTrigger.whenActive(liftDownCommand);

        //44. Go to command teleop and create a lift mechanism
    }

    @Override
    public void createBase(){
        telemetry.addLine("Lift createBase");
        //34. Create the motor
        //Make sure the device name from the OpMode matches what's in the hardware map
        //the GoBilda part of this may not be needed based on the attached motor
        //FtcLib has special features for GoBuilda motors
        LM = new MotorEx(hwMap, deviceName, Motor.GoBILDA.RPM_312);
        telemetry.addData("Li", LM);
        telemetry.update();

        //35. create the subsystem, given that telemetry is used everywhere, use the constructor that sets it
        liftSubsystem = new LiftSubsystem(LM, telemetry);
        //36. set what the motor would do when it has no power
        liftSubsystem.setZeroPowerBehavoir(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //37.not using encoders at this point but recommend stopping them
        liftSubsystem.stopResetEncoder();
        //38. set the direction of the motor, ideally this is tested while not on the lift
        liftSubsystem.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public LiftUpCommand createLiftUpCommand(){
        return new LiftUpCommand(liftSubsystem, liftUpDS, telemetry);
    }

    public LiftDownCommand createLiftDownCommand(){
        return new LiftDownCommand(liftSubsystem, liftDownDS, telemetry);
    }

    public LiftSubsystem getLiftSubsystem() {
        return liftSubsystem;
    }
}
