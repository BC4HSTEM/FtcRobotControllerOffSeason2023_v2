package org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.subsystems.PixelGrabberSubsystem;

public class PixelGrabberCloseCommand extends CommandBase{
    private PixelGrabberSubsystem grabberSubsystem;
    private Telemetry telemetry;

    //private double position = 0.5;

    public PixelGrabberCloseCommand(PixelGrabberSubsystem grabberSubsystem){
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    public PixelGrabberCloseCommand(PixelGrabberSubsystem grabberSubsystem, Telemetry telemetry){
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