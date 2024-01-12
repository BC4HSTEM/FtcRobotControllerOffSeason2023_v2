package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;
import org.openftc.easyopencv.OpenCvPipeline;
public class DetectTEPosition extends CommandBase {

    private final PositionIdentifierSubsystem positionIdentifierSubsystem;

    private Telemetry telemetry;
    private Positions.TEPosition position;

    public DetectTEPosition(PositionIdentifierSubsystem subsystem){
        positionIdentifierSubsystem = subsystem;
        addRequirements(positionIdentifierSubsystem);
    }

    public DetectTEPosition(PositionIdentifierSubsystem subsystem, Telemetry telemetry){
        //telemetry.addData("DetectTSEPosition", level);
        positionIdentifierSubsystem = subsystem;
        this.telemetry = telemetry;

        addRequirements(positionIdentifierSubsystem);
    }

    @Override
    public void initialize(){

        positionIdentifierSubsystem.detectTEPosition();
        //telemetry.addData("We are initialize", "detectPosition");
    }

    @Override
    public void execute() {
        //telemetry.addData("We are execute position set", positionIdentifierSubsystem.isPositionSet());
        //telemetry.addData("Which Position", positionIdentifierSubsystem.getPosition());
        //telemetry.update();
    }

    @Override
    public boolean isFinished() {
        //telemetry.addData("We are finished position set", positionIdentifierSubsystem.isPositionSet());
        //telemetry.addData("Which Position", positionIdentifierSubsystem.getPosition());
        //telemetry.update();
        return positionIdentifierSubsystem.isPositionSet();
    }
}
