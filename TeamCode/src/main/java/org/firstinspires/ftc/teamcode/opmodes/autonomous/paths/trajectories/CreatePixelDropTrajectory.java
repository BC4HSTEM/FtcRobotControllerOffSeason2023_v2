package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.Alliance;
import org.firstinspires.ftc.teamcode.globals.PixelTrajectory;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.globals.Side;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems.roadrunner.MecanumDriveSubsystem;

public class CreatePixelDropTrajectory {
    private Vector2d redNonStageSideLeft = new Vector2d(-46,-30);
    private Vector2d redNonStageSideMiddle = new Vector2d(-36,-55);
    private Vector2d redNonStageSideRight = new Vector2d(-27,-29);

    private Vector2d redStageSideLeft = new Vector2d(-46,-30);
    private Vector2d redStageSideMiddle = new Vector2d(-36,-26);
    private Vector2d redStageSideRight = new Vector2d(-27,-29);

    private Vector2d blueNonStageSideLeft = new Vector2d(-46,-30);
    private Vector2d blueNonStageSideMiddle = new Vector2d(-36,-26);
    private Vector2d blueNonStageSideRight = new Vector2d(-27,-29);

    private Vector2d blueStageSideLeft = new Vector2d(-46,-30);
    private Vector2d blueStageSideMiddle = new Vector2d(-36,-55);
    private Vector2d blueStageSideRight = new Vector2d(-27,-29);
    private Vector2d finalPosition = redNonStageSideRight;

    private final MecanumDriveSubsystem drive;
    private final Pose2d previousTrajEnd;
    private final Telemetry telemetry;

    Trajectory pixelTraj;

    TrajectoryFollowerCommand followPixel;
    public CreatePixelDropTrajectory(MecanumDriveSubsystem drive, Pose2d previousTrajEnd, Telemetry telemetry){
        this.drive = drive;
        this.previousTrajEnd = previousTrajEnd;
        this.telemetry = telemetry;
    }

    public Trajectory createTrajectory(){
        telemetry.addData("Tag snapshot id in Park Command:", Positions.getInstance().getTEPosition());

        Positions.TEPosition psPosition = Positions.getInstance().getTEPosition();
        if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED)
        {
            if(Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
                if(psPosition == null || psPosition == Positions.TEPosition.POSITION_LEFT){
                    finalPosition = redNonStageSideLeft;
                }else if(psPosition == Positions.TEPosition.POSITION_MIDDLE){
                    finalPosition = redNonStageSideMiddle;
                }else{
                    finalPosition = redNonStageSideRight;
                }
            }
            else if(Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
                if(psPosition == null || psPosition == Positions.TEPosition.POSITION_LEFT){
                    finalPosition = redStageSideLeft;
                }else if(psPosition == Positions.TEPosition.POSITION_MIDDLE){
                    finalPosition = redStageSideMiddle;
                }else{
                    finalPosition = redStageSideRight;
                }
            }

        }
        else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE)
        {
            if(Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
                if(psPosition == null || psPosition == Positions.TEPosition.POSITION_LEFT){
                    finalPosition = blueNonStageSideLeft;
                }else if(psPosition == Positions.TEPosition.POSITION_MIDDLE){
                    finalPosition = blueNonStageSideMiddle;
                }else{
                    finalPosition = blueNonStageSideRight;
                }
            }
            else if(Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
                if(psPosition == null || psPosition == Positions.TEPosition.POSITION_LEFT){
                    finalPosition = blueStageSideLeft;
                }else if(psPosition == Positions.TEPosition.POSITION_MIDDLE){
                    finalPosition = blueStageSideMiddle;
                }else{
                    finalPosition = blueStageSideRight;
                }
            }

        }


        telemetry.addData("Final Position:", finalPosition.toString());
        telemetry.update();


        pixelTraj = drive.trajectoryBuilder(previousTrajEnd)
                .lineToConstantHeading(finalPosition)
                .build();

        return pixelTraj;

    }

    public Trajectory getPixelTraj(){
        return pixelTraj;
    }

}
