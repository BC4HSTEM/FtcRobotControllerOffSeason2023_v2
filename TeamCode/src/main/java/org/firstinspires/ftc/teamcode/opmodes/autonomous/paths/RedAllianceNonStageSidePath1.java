package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.globals.Junctions;
import org.firstinspires.ftc.teamcode.globals.PixelTrajectory;
import org.firstinspires.ftc.teamcode.mechanisms.arm.CreateArmMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.ParkCommandRedSideStage;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.RunToPixelDropLocationCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems.roadrunner.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.drone_launcher.CreateDroneLauncherMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.grabber.CreateGrabberMechanism;

import org.firstinspires.ftc.teamcode.mechanisms.grabber.commands.GrabberCloseCommand;

import org.firstinspires.ftc.teamcode.mechanisms.grabber.commands.GrabberCommand;
import org.firstinspires.ftc.teamcode.mechanisms.grabber_wrist.CreateGrabberWristMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.lift.CreateLiftMechanism;

import org.firstinspires.ftc.teamcode.mechanisms.lift.subsystems.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.mechanisms.sleevereader.CreateSleeveReaderMechanism;
//import org.firstinspires.ftc.teamcode.mechanisms.sleevereader.commands.ReadSleeveCommand;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.commands.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.CreatePixelGrabberMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberLeftCommand;
import org.firstinspires.ftc.teamcode.mechanisms.pixel_grabber.commands.PixelGrabberRightCommand;

public class RedAllianceNonStageSidePath1 {

    private MecanumDriveSubsystem drive;
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

    private WaitCommand waitCommand1;

    private FtcDashboard dashboard;

    private SequentialCommandGroup liftGroup;
    private SequentialCommandGroup grabberGroup;

    private final Pose2d startPose;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    private CreateLiftMechanism createLiftMechanism;
    private CreateGrabberMechanism createGrabberMechanism;

    private PixelGrabberLeftCommand grabberOpenLeftCommand;
    private PixelGrabberRightCommand grabberOpenRightCommand;

    private TurnCommand turnCommand;

    private ParkCommandRedSideStage parkCommand;

    private RunToPixelDropLocationCommand runToPixelDropLocationCommand;


    private Trajectory traj3;

    SequentialCommandGroup sg;
    SequentialCommandGroup open;
    SequentialCommandGroup close;


    public RedAllianceNonStageSidePath1(HardwareMap hwMap, Pose2d sp, Telemetry telemetry){
        this.hwMap = hwMap;
        startPose = sp;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public RedAllianceNonStageSidePath1(HardwareMap hwMap, Pose2d sp, FtcDashboard db, Telemetry telemetry){
        this.hwMap = hwMap;
        startPose = sp;
        dashboard = db;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public void createPath(){
        drive.setPoseEstimate(startPose);


        CreateArmMechanism createArmMechanism = new CreateArmMechanism(hwMap, "arm", telemetry);
        createArmMechanism.create();

        CreatePixelGrabberMechanism createPixelGrabberMechanism = new CreatePixelGrabberMechanism(hwMap, "pixel_grabber", telemetry);
        createPixelGrabberMechanism.createAuto();

        CreateDroneLauncherMechanism createDroneLauncherMechanism = new CreateDroneLauncherMechanism(hwMap, "drone_Launch", telemetry);
        createDroneLauncherMechanism.createAuto();

        CreateGrabberWristMechanism createGrabberWristMechanism = new CreateGrabberWristMechanism(hwMap, "wrist_Motion", telemetry);
        createGrabberWristMechanism.createAuto();

        //CreateGrabberMechanism grabberMechanism = new CreateGrabberMechanism(hwMap, "grab", telemetry);
        //grabberMechanism.createAuto();

        //createLiftMechanism = new CreateLiftMechanism(hwMap, "lift", telemetry);
        //createLiftMechanism.createAuto();

        //LiftSubsystem liftSubsystem = createLiftMechanism.getLiftSubsystem();


        //turnCommand = new TurnCommand(drive, Math.toRadians(-40));
        waitCommand1 = new WaitCommand (1000);
        //holds Purple Pixels
        grabberOpenLeftCommand = createPixelGrabberMechanism.createGrabberLeftCommand();
        //Holds Yellow Pixel
        grabberOpenRightCommand = createPixelGrabberMechanism.createGrabberRightCommand();


        runToPixelDropLocationCommand = new RunToPixelDropLocationCommand(drive, startPose, telemetry);

        Trajectory traj1 = drive.trajectoryBuilder(PixelTrajectory.getInstance().getPixelTraj().end())
                /*.addDisplacementMarker(() -> {
                    turnCommand.schedule();
                })*/
                .lineToLinearHeading(new Pose2d(-36,-26, Math.toRadians(270)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //.lineTo(new Vector2d(-41, 52))
                .lineToLinearHeading(new Pose2d(36,55,Math.toRadians(2)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //.lineToConstantHeading(new Vector2d(-37, 2))
                .lineToConstantHeading(new Vector2d(37, -3))
                .build();

        //LINE UP WITH CONE STACK
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d( 37, 6.5,Math.toRadians(2)))
                .build();

        //START PICKING UP AND DROPPING OFF CONES
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToConstantHeading(new Vector2d( 58.5, 6.5))
                .build();

        Trajectory traj6a = drive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d( 36, 4, Math.toRadians(225)))
                .build();

        Trajectory traj6b = drive.trajectoryBuilder(traj6a.end())
                .lineToLinearHeading(new Pose2d( 29.5, 3,  Math.toRadians(225)))
                .build();

        //CONE DROP

        Trajectory traj7 = drive.trajectoryBuilder(traj6b.end())
                .lineToLinearHeading(new Pose2d( 35, 10, Math.toRadians(180)))
                .build();

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToLinearHeading(new Pose2d( 39, 14, Math.toRadians(225)))
                .build();

        //CONE DROP

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToLinearHeading(new Pose2d( 37, 14,Math.toRadians(2)))
                .build();


        parkCommand = new ParkCommandRedSideStage(drive, traj7.end(), telemetry);

        follower1 = new TrajectoryFollowerCommand(drive,traj1);
        follower2 = new TrajectoryFollowerCommand(drive,traj2);
        follower3 = new TrajectoryFollowerCommand(drive,traj3);
        follower4 = new TrajectoryFollowerCommand(drive,traj4);
        follower5 = new TrajectoryFollowerCommand(drive,traj5);
        follower6a = new TrajectoryFollowerCommand(drive,traj6a);
        follower6b = new TrajectoryFollowerCommand(drive,traj6b);
        follower7 = new TrajectoryFollowerCommand(drive,traj7);
        follower8 = new TrajectoryFollowerCommand(drive,traj8);
        follower9 = new TrajectoryFollowerCommand(drive,traj9);
    }

    public void execute(CommandOpMode commandOpMode){
        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(

                follower1, grabberOpenLeftCommand));
    }

}
