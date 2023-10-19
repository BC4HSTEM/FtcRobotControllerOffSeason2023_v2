package org.firstinspires.ftc.teamcode.mechanisms.grabber.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.subsystems.GrabberSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

//9. extend CommandBase
public class GrabberCommand extends CommandBase{
    //10.Define the subsystem(s) that this Command requires
    private GrabberSubsystem grabberSubsystem;
    private Telemetry telemetry;

    public GrabberCommand(GrabberSubsystem grabberSubsystem){
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    public GrabberCommand(GrabberSubsystem grabberSubsystem, Telemetry telemetry){
        //11. Set the subsystem
        this.grabberSubsystem = grabberSubsystem;
        this.telemetry = telemetry;
        //12. let ftclib know this command is dependent on this subsystem
        addRequirements(grabberSubsystem);
    }

    //13. no need to put this in execute as this only needs to run once so initialize is the best place for it

    @Override
    public void initialize(){
        //14. call openGrabber on subsystem
        telemetry.addLine("grabber initialize");
        telemetry.addData("grabber position", grabberSubsystem.getPosition());
        telemetry.addData("grabber open position", grabberSubsystem.getOpenPosition());
        telemetry.update();
        grabberSubsystem.openGrabber();
    }

    //15. isFinished may only be needed to assist with autonomous
    @Override
    public boolean isFinished(){
        return grabberSubsystem.getPosition() <= grabberSubsystem.getOpenPosition();
    }

    //16. Create GrabberCloseCommand using the above structure, the code on initialize will change slightly
    //and so will isFinished
    //17. Go to CreateGrabberMechanism
}