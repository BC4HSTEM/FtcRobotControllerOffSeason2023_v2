package org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.ParkingSpot;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems.roadrunner.MecanumDriveSubsystem;

public class ParkCommandRedBlue extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final Pose2d previousTrajEnd;
    private final Telemetry telemetry;

    private Vector2d position3 = new Vector2d(15,8);
    private Vector2d position2 = new Vector2d(37,8);
    private Vector2d postion1 = new Vector2d(58,8);
    private Vector2d finalPosition = position3;

    TrajectoryFollowerCommand followPark;

    public ParkCommandRedBlue(MecanumDriveSubsystem drive, Pose2d previousTrajEnd, Telemetry telemetry) {
        this.drive = drive;
        this.previousTrajEnd = previousTrajEnd;
        this.telemetry = telemetry;

        //addRequirements(drive);
    }

    @Override
    public void initialize() {

        telemetry.addData("Tag snapshot id in Park Command:", ParkingSpot.getInstance().getSelectedParkingSpotLocaton());

        ParkingSpot.ParkingSpotLocation psLocation = ParkingSpot.getInstance().getSelectedParkingSpotLocaton();
        if(psLocation == null || psLocation == ParkingSpot.ParkingSpotLocation.ONE){
            finalPosition = postion1;
        }else if(psLocation == ParkingSpot.ParkingSpotLocation.TWO){
            finalPosition = position2;
        }else{
            finalPosition = position3;
        }

        telemetry.addData("Final Position:", finalPosition.toString());
        telemetry.update();


        Trajectory parkTraj = drive.trajectoryBuilder(previousTrajEnd)
                .lineToConstantHeading(finalPosition)
                .build();

        followPark = new TrajectoryFollowerCommand(drive,parkTraj);

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        followPark.schedule();
        //drive.stop();

    }

    @Override
    public boolean isFinished() {
        return followPark.isFinished();
    }
}
