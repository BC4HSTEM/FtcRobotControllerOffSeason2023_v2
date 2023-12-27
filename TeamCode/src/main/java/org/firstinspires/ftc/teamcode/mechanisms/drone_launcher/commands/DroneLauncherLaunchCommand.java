package org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.subsystems.DroneLauncherSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.subsystems.PixelGrabberSubsystem;

public class DroneLauncherLaunchCommand extends CommandBase {

    DroneLauncherSubsystem droneLauncherSubsystem;
    Telemetry telemetry;
    public DroneLauncherLaunchCommand(DroneLauncherSubsystem dlSubsystem, Telemetry telemetry){
        this.droneLauncherSubsystem = dlSubsystem;
        this.telemetry = telemetry;

        addRequirements(droneLauncherSubsystem);
    }

    @Override
    public void initialize(){

        droneLauncherSubsystem.LaunchDrone();

    }
}
