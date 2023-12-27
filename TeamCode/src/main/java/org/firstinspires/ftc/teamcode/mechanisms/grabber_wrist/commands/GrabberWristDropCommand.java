package org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.subsystems.DroneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.subsystems.GrabberWristSubsystem;

public class GrabberWristDropCommand extends CommandBase {

    GrabberWristSubsystem grabberWristSubsystem;
    Telemetry telemetry;
    public GrabberWristDropCommand(GrabberWristSubsystem gwSubsystem, Telemetry telemetry){
        grabberWristSubsystem = gwSubsystem;
        this.telemetry = telemetry;

        addRequirements(grabberWristSubsystem);
    }

    @Override
    public void initialize(){

        grabberWristSubsystem.DropPixel();

    }
}
