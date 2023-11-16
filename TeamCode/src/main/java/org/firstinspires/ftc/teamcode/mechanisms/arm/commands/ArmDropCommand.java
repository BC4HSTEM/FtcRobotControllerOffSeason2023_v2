package org.firstinspires.ftc.teamcode.mechanisms.arm.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class ArmDropCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private Telemetry telemetry;

    public static int dropTargetPosition = 600;

    public ArmDropCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    public ArmDropCommand(ArmSubsystem armSubsystem, Telemetry telemetry){
        this.armSubsystem = armSubsystem;
        this.telemetry = telemetry;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setTargetPosition(dropTargetPosition);
    }


}
