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
    private Pose2d redNonStageSideLeft = new Pose2d(-36,-40, Math.toRadians(150));
    private Pose2d redNonStageSideMiddle = new Pose2d(-36,-32, Math.toRadians(90));
    private Pose2d redNonStageSideRight = new Pose2d(-30,-36, Math.toRadians(50));

    private Pose2d redStageSideLeft = new Pose2d(-36,-50, Math.toRadians(90));
    private Pose2d redStageSideMiddle = new Pose2d(-36,-50, Math.toRadians(90));
    private Pose2d redStageSideRight = new Pose2d(-36,-50, Math.toRadians(90));

    private Pose2d blueNonStageSideLeft = new Pose2d(-36,40, Math.toRadians(270));
    private Pose2d blueNonStageSideMiddle = new Pose2d(-36,40, Math.toRadians(270));
    private Pose2d blueNonStageSideRight = new Pose2d(-36,-50, Math.toRadians(90));

    private Pose2d blueStageSideLeft = new Pose2d(-36,-50, Math.toRadians(90));
    private Pose2d blueStageSideMiddle = new Pose2d(-36,-50, Math.toRadians(90));
    private Pose2d blueStageSideRight = new Pose2d(-36,-50, Math.toRadians(90));
    private Pose2d finalPosition = redNonStageSideRight;



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
                .lineToLinearHeading(finalPosition)
                .build();

        return pixelTraj;

    }

    public Trajectory getPixelTraj(){
        return pixelTraj;
    }

}
