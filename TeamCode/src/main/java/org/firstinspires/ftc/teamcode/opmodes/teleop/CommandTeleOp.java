package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.arm.CreateArmMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.CreateDriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.CreateDroneLauncherMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.CreateGrabberMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.CreateGrabberWristMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.lift.CreateLiftMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.CreatePixelGrabberMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.CreatePositionIdentifierMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;


@TeleOp(name="Command Combined TeleOp")
public class CommandTeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        CreateDriveTrainMechanism createDriveTrain = new CreateDriveTrainMechanism(hardwareMap, "drive", driver1, telemetry, true);
        //45.CreateLiftMechanism and be sure to pass in telemetry and true for autoCreate
        //CreateLiftMechanism createLift = new CreateLiftMechanism(hardwareMap, "lift", driver1, telemetry, true);
        //33. Create the GrabberMechanism
        //CreateGrabberMechanism createGrabber = new CreateGrabberMechanism(hardwareMap, "grab", driver1, telemetry, true);

        CreateArmMechanism createArmMechanism = new CreateArmMechanism(hardwareMap, "arm", driver1, telemetry, true);
        CreatePixelGrabberMechanism createPixelGrabber = new CreatePixelGrabberMechanism(hardwareMap, "pixel_grabber", driver1, telemetry, true);
        CreateDroneLauncherMechanism createDroneLauncherMechanism = new CreateDroneLauncherMechanism(hardwareMap, "drone_Launch", driver1, telemetry, true);
        CreateGrabberWristMechanism createGrabberWristMechanism = new CreateGrabberWristMechanism(hardwareMap, "wrist_Motion", driver1, telemetry, true);
        //CreatePositionIdentifierMechanism createPositionIdentifierMechanism = new CreatePositionIdentifierMechanism(hardwareMap, "web_cam", driver1, telemetry, true);

    }
    public void execute(){

    }
}
