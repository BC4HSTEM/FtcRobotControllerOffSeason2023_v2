package org.firstinspires.ftc.teamcode.mechanisms.grabber.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.subsystems.GrabberSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class GrabberCommand extends CommandBase{
    private GrabberSubsystem grabberSubsystem;
    private Telemetry telemetry;

    public GrabberCommand(GrabberSubsystem grabberSubsystem){
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    public GrabberCommand(GrabberSubsystem grabberSubsystem, Telemetry telemetry){
        this.grabberSubsystem = grabberSubsystem;
        this.telemetry = telemetry;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize(){
        grabberSubsystem.openGrabber();
    }

    @Override
    public boolean isFinished(){
        return grabberSubsystem.getPosition() <= grabberSubsystem.getOpenPosition();
    }

}