package org.firstinspires.ftc.teamcode.mechanisms.grabber.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.subsystems.GrabberSubsystem;

import java.util.function.DoubleSupplier;

public class GrabberCloseCommand extends CommandBase{
    private GrabberSubsystem grabberSubsystem;
    private Telemetry telemetry;

    //private double position = 0.5;

    public GrabberCloseCommand(GrabberSubsystem grabberSubsystem){
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    public GrabberCloseCommand(GrabberSubsystem grabberSubsystem, Telemetry telemetry){
        this.grabberSubsystem = grabberSubsystem;
        this.telemetry = telemetry;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize(){
        telemetry.addLine("grabber close initialize");
        telemetry.addData("grabber position", grabberSubsystem.getPosition());
        telemetry.addData("grabber close position", grabberSubsystem.getClosePosition());
        telemetry.update();
        grabberSubsystem.closeGrabber();
    }


    @Override
    public boolean isFinished(){
        return grabberSubsystem.getPosition() >= grabberSubsystem.getClosePosition();
    }

}