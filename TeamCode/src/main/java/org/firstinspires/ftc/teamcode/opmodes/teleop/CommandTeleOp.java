package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.CreateDriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.lift.CreateLiftMechanism;


@TeleOp(name="Command Combined TeleOp")
public class CommandTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        CreateDriveTrainMechanism createDriveTrain = new CreateDriveTrainMechanism(hardwareMap, "drive", driver1, telemetry, true);
        //45.CreateLiftMechanism and be sure to pass in telemetry and true for autoCreate
        CreateLiftMechanism createLift = new CreateLiftMechanism(hardwareMap, "lift", driver1, telemetry, true);
    }
    public void execute(){

    }
}
