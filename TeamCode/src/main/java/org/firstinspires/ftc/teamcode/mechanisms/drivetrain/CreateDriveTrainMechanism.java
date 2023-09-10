package org.firstinspires.ftc.teamcode.mechanisms.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.CreateMechanismBase;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems.DriveSubsystem;

public class CreateDriveTrainMechanism extends CreateMechanismBase {

    private DriveSubsystem driveSubsystem;
    private Motor fl, bl, fr, br;

    private DriveCommand driveCommand;
    private SlowDriveCommand slowDriveCommand;

    public CreateDriveTrainMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry){
        super(hwMap, deviceName, op, telemetry);

    }

    public CreateDriveTrainMechanism(HardwareMap hwMap, String deviceName, GamepadEx op, Telemetry telemetry, boolean autoCreate){
        super(hwMap, deviceName, op, telemetry, autoCreate);


    }

    public CreateDriveTrainMechanism(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        super(hwMap, deviceName, telemetry);

    }

    @Override
    public void create(){

        createBase();

        op.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .toggleWhenActive(slowDriveCommand, driveCommand);

    }

    @Override
    public void createBase(){
        fl = new Motor(hwMap, "frontLeft");
        bl = new Motor(hwMap, "backLeft");
        fr = new Motor(hwMap, "frontRight");
        br = new Motor(hwMap, "backRight");

        driveSubsystem = new DriveSubsystem(fl, fr, bl, br);

        driveCommand = createDriveCommand();
        slowDriveCommand = createSlowDriveCommand();

        driveSubsystem.setDefaultCommand(driveCommand);
    }

    public DriveCommand createDriveCommand(){
        return new DriveCommand(driveSubsystem, op::getLeftX, op::getLeftY, op::getRightX);
    }

    public SlowDriveCommand createSlowDriveCommand(){
        return new SlowDriveCommand(driveSubsystem, op::getLeftX, op::getLeftY, op::getRightX);
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }
}
