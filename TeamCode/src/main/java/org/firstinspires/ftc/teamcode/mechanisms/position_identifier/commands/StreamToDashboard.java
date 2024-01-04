package org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.subsystems.PositionIdentifierSubsystem;

public class StreamToDashboard extends CommandBase {
    private final PositionIdentifierSubsystem positionIdentifierSubsystem;


    public StreamToDashboard(PositionIdentifierSubsystem subsystem){
        positionIdentifierSubsystem = subsystem;
        addRequirements(positionIdentifierSubsystem);

    }


    @Override
    public void initialize(){
        //positionIdentifierSubsystem.StreamToDashboard();
    }

}
