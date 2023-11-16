package org.firstinspires.ftc.teamcode.mechanisms.arm;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmDropCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmPickUpCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;

public class CreateArmMechanism extends CreateMechanismBase {

    private ArmSubsystem armSubsystem;
    private ArmDropCommand armDropCommand;
    private ArmPickUpCommand armPickUpCommand;

    private DcMotorEx arm;

    public CreateArmMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);
    }
    
    public CreateArmMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);
    }

    public CreateArmMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){

        //33. after setting up our triggers we can now creates our subsystem
        createBase();

        //39. created commands

        //40. set motor to run without encoders
        armSubsystem.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //41. instead of creating the command right here, we created a function for it so we
        //can reuse the code for Auto
        armDropCommand = createDropCommand();
        armPickUpCommand = createPickUpCommand();

        op.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(armDropCommand);
        op.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(armPickUpCommand);
        armSubsystem.setDefaultCommand(armPickUpCommand);

    }

    @Override
    public void createBase(){
        telemetry.addLine("Arm createBase");

        arm = hwMap.get(DcMotorEx.class, deviceName);
        telemetry.addData("Arm", arm);
        telemetry.update();

        //35. create the subsystem, given that telemetry is used everywhere, use the constructor that sets it
        armSubsystem = new ArmSubsystem(arm, telemetry, true);
        //36. set what the motor would do when it has no power
        armSubsystem.setZeroPowerBehavoir(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //37.not using encoders at this point but recommend stopping them
        armSubsystem.stopResetEncoder();
        //38. set the direction of the motor, ideally this is tested while not on the lift
        armSubsystem.setDirection(DcMotorEx.Direction.FORWARD);

        //op.getGamepadButton(GamepadKeys.Button.Y).whenPressed(armPickUpCommand);


    }

    private ArmDropCommand createDropCommand(){
        return new ArmDropCommand(armSubsystem, telemetry);
    }

    private ArmPickUpCommand createPickUpCommand(){
        return new ArmPickUpCommand(armSubsystem, telemetry);
    }


}
