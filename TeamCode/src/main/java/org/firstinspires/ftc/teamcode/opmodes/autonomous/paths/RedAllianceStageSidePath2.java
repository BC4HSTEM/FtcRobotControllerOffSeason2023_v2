package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.mechanisms.arm.CreateArmMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmDropCommand;
import org.firstinspires.ftc.teamcode.mechanisms.arm.commands.ArmMidDropCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.CreateDroneLauncherMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.CreateGrabberMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.CreateGrabberWristMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.commands.GrabberWristDropCommand;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.commands.GrabberWristPickUpCommand;
import org.firstinspires.ftc.teamcode.mechanisms.lift.CreateLiftMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.CreatePixelGrabberMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberLeftCloseCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberLeftOpenCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberRightCloseCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberRightOpenCommand;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.CreatePositionIdentifierMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.commands.DetectTEPosition;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.trajectories.CreatePixelDropTrajectory;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.trajectories.CreatePixelPlaceTrajectory;

public class RedAllianceStageSidePath2 {

    private MecanumDriveSubsystem drive;

    private TrajectoryFollowerCommand followPixel;

    private TrajectoryFollowerCommand followPlacePixel;
    private TrajectoryFollowerCommand follower1;
    private TrajectoryFollowerCommand follower2;
    private TrajectoryFollowerCommand follower3;
    private TrajectoryFollowerCommand follower4;
    private TrajectoryFollowerCommand follower5;
    private TrajectoryFollowerCommand follower6a;
    private TrajectoryFollowerCommand follower6b;
    private TrajectoryFollowerCommand follower7;
    private TrajectoryFollowerCommand follower8;
    private TrajectoryFollowerCommand follower9;
    private TrajectoryFollowerCommand follower10;
    private TrajectoryFollowerCommand followPark;
    private TrajectoryFollowerCommand follower11;

    private WaitCommand waitCommand2000;
    private WaitCommand waitCommand1000;

    private FtcDashboard dashboard;

    private SequentialCommandGroup liftGroup;
    private SequentialCommandGroup grabberGroup;

    private final Pose2d startPose;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    private CreateLiftMechanism createLiftMechanism;
    private CreateGrabberMechanism createGrabberMechanism;

    private PixelGrabberLeftOpenCommand grabberCloseLeftCommand;
    private PixelGrabberLeftCloseCommand grabberOpenLeftCommand;
    private PixelGrabberRightOpenCommand grabberOpenRightCommand;

    private PixelGrabberRightCloseCommand grabberCloseRightCommand;

    private GrabberWristDropCommand grabberWristDropCommand;
    private GrabberWristPickUpCommand grabberWristPickUpCommand;
    private DetectTEPosition detectTEPositionCommand;


    private ArmMidDropCommand armMidDropCommand;
    private ArmDropCommand armDropCommand;




    private TurnCommand turnCommand;



    CreatePositionIdentifierMechanism createPositionIdentifierMechanism;

    private Trajectory traj3;

    SequentialCommandGroup sg;
    SequentialCommandGroup open;
    SequentialCommandGroup close;


    public RedAllianceStageSidePath2(HardwareMap hwMap, CreatePositionIdentifierMechanism mechanism, Pose2d sp, Telemetry telemetry){
        this.hwMap = hwMap;
        createPositionIdentifierMechanism = mechanism;
        startPose = sp;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public RedAllianceStageSidePath2(HardwareMap hwMap, CreatePositionIdentifierMechanism mechanism,Pose2d sp, FtcDashboard db, Telemetry telemetry){
        this.hwMap = hwMap;
        createPositionIdentifierMechanism = mechanism;
        startPose = sp;
        dashboard = db;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public void createPath(){

    }

    public void execute(CommandOpMode commandOpMode){

        telemetry.update();
        drive.setPoseEstimate(startPose);

        CreateArmMechanism createArmMechanism = new CreateArmMechanism(hwMap, "arm", telemetry);
        createArmMechanism.createAuto();

        CreatePixelGrabberMechanism createPixelGrabberMechanism = new CreatePixelGrabberMechanism(hwMap, "pixel_grabber", telemetry);
        createPixelGrabberMechanism.createAuto();

        CreateDroneLauncherMechanism createDroneLauncherMechanism = new CreateDroneLauncherMechanism(hwMap, "drone_Launch", telemetry);
        createDroneLauncherMechanism.createAuto();

        CreateGrabberWristMechanism createGrabberWristMechanism = new CreateGrabberWristMechanism(hwMap, "wrist_Motion", telemetry);
        createGrabberWristMechanism.createAuto();

        createPositionIdentifierMechanism = new CreatePositionIdentifierMechanism(hwMap, "Webcam 1", telemetry);
        createPositionIdentifierMechanism.createAuto();

        waitCommand2000 = new WaitCommand (2000);
        waitCommand1000 = new WaitCommand (1000);
        detectTEPositionCommand = createPositionIdentifierMechanism.getDetectTEPositionCommand();
        //holds Purple Pixels
        grabberCloseLeftCommand = createPixelGrabberMechanism.createGrabberLeftCloseCommand();
        grabberOpenLeftCommand = createPixelGrabberMechanism.createGrabberLeftOpenCommand();
        //Holds Yellow Pixel
        grabberOpenRightCommand = createPixelGrabberMechanism.createGrabberRightOpenCommand();

        grabberWristDropCommand = createGrabberWristMechanism.createGrabberWristDropCommand();
        grabberWristPickUpCommand = createGrabberWristMechanism.createGrabberWristPickUpCommand();

        armMidDropCommand = createArmMechanism.createMidDropCommand();
        armDropCommand = createArmMechanism.createDropCommand();

        GrabberWristPickUpCommand grabberWristPickUpCommand2 = createGrabberWristMechanism.createGrabberWristPickUpCommand();

        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(
                /*detectTEPositionCommand.andThen(new InstantCommand(()->{
                    telemetry.addData("Selection Position Stage Side Red 2", Positions.getInstance().getTEPosition());
                    telemetry.update();
                }))));*/
                grabberWristPickUpCommand,
                detectTEPositionCommand.andThen(new InstantCommand(()->{

                    telemetry.addData("Selection Position Stage Side Red", createPositionIdentifierMechanism.getPosition());
                    telemetry.update();

                    CreatePixelDropTrajectory createPixelDropTrajectory = new CreatePixelDropTrajectory(drive, createPositionIdentifierMechanism,startPose, telemetry);
                    Trajectory pixelTraj = createPixelDropTrajectory.createTrajectory();



                    if(createPositionIdentifierMechanism.getPosition() != Positions.TEPosition.POSITION_LEFT){
                        Trajectory traj1 = drive.trajectoryBuilder(pixelTraj.end())

                                .lineToLinearHeading(new Pose2d(14, -53, Math.toRadians(0)))
                                .build();

                        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                                .lineToLinearHeading(new Pose2d(48, -53, Math.toRadians(0)))
                                .build();

                        /*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())

                                .lineToLinearHeading(new Pose2d(50, -32, Math.toRadians(0)))
                                .build();*/

                        CreatePixelPlaceTrajectory createPixelPlaceTrajectory = new CreatePixelPlaceTrajectory(drive, createPositionIdentifierMechanism, traj2.end(), telemetry);
                        Trajectory pixelPlaceTraj = createPixelPlaceTrajectory.createTrajectory();

                        followPixel = new TrajectoryFollowerCommand(drive,pixelTraj);

                        follower1 = new TrajectoryFollowerCommand(drive,traj1);
                        follower2 = new TrajectoryFollowerCommand(drive,traj2);
                        follower3 = new TrajectoryFollowerCommand(drive,traj3);

                        followPlacePixel = new TrajectoryFollowerCommand(drive,pixelPlaceTraj);

                        new SequentialCommandGroup(followPixel, grabberOpenRightCommand,grabberWristDropCommand,
                                follower1, follower2,armDropCommand, followPlacePixel, grabberWristPickUpCommand2, waitCommand1000, grabberOpenLeftCommand).schedule();
                    }
                    else{

                        Trajectory traj1 = drive.trajectoryBuilder(pixelTraj.end())

                                .lineToLinearHeading(new Pose2d(14, -54, Math.toRadians(180)))
                                .build();

                        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                                .lineToLinearHeading(new Pose2d(14, -53, Math.toRadians(0)))
                                .build();
                        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())

                                .lineToLinearHeading(new Pose2d(48, -53, Math.toRadians(0)))
                                .build();

                        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())

                                .lineToLinearHeading(new Pose2d(48, -32, Math.toRadians(0)))
                                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                                .build();
                        CreatePixelPlaceTrajectory createPixelPlaceTrajectory = new CreatePixelPlaceTrajectory(drive, createPositionIdentifierMechanism, traj3.end(), telemetry);
                        Trajectory pixelPlaceTraj = createPixelPlaceTrajectory.createTrajectory();

                        followPixel = new TrajectoryFollowerCommand(drive,pixelTraj);

                        follower1 = new TrajectoryFollowerCommand(drive,traj1);
                        follower2 = new TrajectoryFollowerCommand(drive,traj2);
                        follower3 = new TrajectoryFollowerCommand(drive,traj3);
                        follower4 = new TrajectoryFollowerCommand(drive,traj4);

                        followPlacePixel = new TrajectoryFollowerCommand(drive,pixelPlaceTraj);

                        new SequentialCommandGroup(followPixel, follower1, grabberOpenRightCommand,grabberWristDropCommand,
                                follower2, follower3,follower4,armDropCommand, followPlacePixel, grabberWristPickUpCommand2, waitCommand1000, grabberOpenLeftCommand).schedule();

                    }



                }))));

    }

}
