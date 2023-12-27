package org.firstinspires.ftc.teamcode.mechanisms.arm.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems.ArmSubsystem;


public class ArmDropPositionCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private Telemetry telemetry;

    public static int dropTargetPosition = 700;

    public ArmDropPositionCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    public ArmDropPositionCommand(ArmSubsystem armSubsystem, Telemetry telemetry){
        this.armSubsystem = armSubsystem;
        this.telemetry = telemetry;


        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setTargetPosition(dropTargetPosition);
    }


}
