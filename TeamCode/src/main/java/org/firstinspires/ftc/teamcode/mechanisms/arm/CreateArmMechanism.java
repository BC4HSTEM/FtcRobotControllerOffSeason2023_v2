package org.firstinspires.ftc.teamcode.mechanisms.arm;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmDropCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmDropPositionCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmPickUpCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmPickUpPositionCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems.ArmSubsystem;

public class CreateArmMechanism extends CreateMechanismBase {

    private ArmSubsystem armSubsystem;
    private ArmDropCommand armDropCommand;
    private ArmPickUpCommand armPickUpCommand;

    private ArmDropPositionCommand armDropPositionCommand;
    private ArmPickUpPositionCommand armPickUpPositionCommand;

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
        //armSubsystem.setTargetPosition(20);

        //41. instead of creating the command right here, we created a function for it so we
        //can reuse the code for Auto
        armDropCommand = createDropCommand();
        armPickUpCommand = createPickUpCommand();

        armDropPositionCommand = createDropPositionCommand();
        armPickUpPositionCommand = createPickUpPositionCommand();

        op.getGamepadButton(GamepadKeys.Button.Y).whenPressed(armDropCommand);
        op.getGamepadButton(GamepadKeys.Button.X).whenPressed(armPickUpCommand);
        //armSubsystem.setDefaultCommand(armPickUpCommand);

    }

    @Override
    public void createBase(){
        telemetry.addLine("Arm createBase");

        arm = hwMap.get(DcMotorEx.class,"arm" );
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

    private ArmDropPositionCommand createDropPositionCommand(){
        return new ArmDropPositionCommand(armSubsystem, telemetry);
    }

    private ArmPickUpPositionCommand createPickUpPositionCommand(){
        return new ArmPickUpPositionCommand(armSubsystem, telemetry);
    }


}
