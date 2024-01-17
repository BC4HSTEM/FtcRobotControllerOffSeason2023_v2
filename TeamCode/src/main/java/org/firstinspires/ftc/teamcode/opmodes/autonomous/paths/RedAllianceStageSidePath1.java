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

public class RedAllianceStageSidePath1 {

    private MecanumDriveSubsystem drive;

    private TrajectoryFollowerCommand followPixel;
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

    private TurnCommand turnCommand120;




    private TurnCommand turnCommand;




    CreatePositionIdentifierMechanism createPositionIdentifierMechanism;
    private Trajectory traj3;

    SequentialCommandGroup sg;
    SequentialCommandGroup open;
    SequentialCommandGroup close;


    public RedAllianceStageSidePath1(HardwareMap hwMap, CreatePositionIdentifierMechanism mechanism, Pose2d sp, Telemetry telemetry){
        this.hwMap = hwMap;
        createPositionIdentifierMechanism = mechanism;
        startPose = sp;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public RedAllianceStageSidePath1(HardwareMap hwMap, CreatePositionIdentifierMechanism mechanism,Pose2d sp, FtcDashboard db, Telemetry telemetry){
        this.hwMap = hwMap;
        createPositionIdentifierMechanism = mechanism;
        startPose = sp;
        dashboard = db;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);

    }

    public void createPath(){

        CreatePositionIdentifierMechanism createPositionIdentifierMechanism = new CreatePositionIdentifierMechanism(hwMap, "Webcam 1", telemetry);
        createPositionIdentifierMechanism.createAuto();

        detectTEPositionCommand = createPositionIdentifierMechanism.getDetectTEPositionCommand();
        /*drive.setPoseEstimate(startPose);


        CreateArmMechanism createArmMechanism = new CreateArmMechanism(hwMap, "arm", telemetry);
        createArmMechanism.createAuto();

        CreatePixelGrabberMechanism createPixelGrabberMechanism = new CreatePixelGrabberMechanism(hwMap, "pixel_grabber", telemetry);
        createPixelGrabberMechanism.createAuto();

        CreateDroneLauncherMechanism createDroneLauncherMechanism = new CreateDroneLauncherMechanism(hwMap, "drone_Launch", telemetry);
        createDroneLauncherMechanism.createAuto();

        CreateGrabberWristMechanism createGrabberWristMechanism = new CreateGrabberWristMechanism(hwMap, "wrist_Motion", telemetry);
        createGrabberWristMechanism.createAuto();

        CreatePositionIdentifierMechanism createPositionIdentifierMechanism = new CreatePositionIdentifierMechanism(hwMap, "Webcam 1", telemetry);
        createPositionIdentifierMechanism.createAuto();

        //CreateGrabberMechanism grabberMechanism = new CreateGrabberMechanism(hwMap, "grab", telemetry);
        //grabberMechanism.createAuto();

        //createLiftMechanism = new CreateLiftMechanism(hwMap, "lift", telemetry);
        //createLiftMechanism.createAuto();

        //LiftSubsystem liftSubsystem = createLiftMechanism.getLiftSubsystem();


        //turnCommand = new TurnCommand(drive, Math.toRadians(-40));
        waitCommand2000 = new WaitCommand (2000);
        detectTEPositionCommand = createPositionIdentifierMechanism.getDetectTEPositionCommand();
        //holds Purple Pixels
        grabberCloseLeftCommand = createPixelGrabberMechanism.createGrabberLeftCloseCommand();
        grabberOpenLeftCommand = createPixelGrabberMechanism.createGrabberLeftOpenCommand();
        //Holds Yellow Pixel
        grabberOpenRightCommand = createPixelGrabberMechanism.createGrabberRightOpenCommand();

        grabberWristDropCommand = createGrabberWristMechanism.createGrabberWristDropCommand();
        grabberWristPickUpCommand = createGrabberWristMechanism.createGrabberWristPickUpCommand();

        turnCommand120 = new TurnCommand(drive, Math.toRadians(120));


        CreatePixelDropTrajectory createPixelDropTrajectory = new CreatePixelDropTrajectory(drive, startPose, telemetry);
        Trajectory pixelTraj = createPixelDropTrajectory.createTrajectory();

        *//*.lineToLinearHeading(new Pose2d(11,-35, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(59, -56, Math.toRadians(0)))*//*


        Trajectory traj1 = drive.trajectoryBuilder(pixelTraj.end())
                *//*.addDisplacementMarker(() -> {
                    turnCommand.schedule();
                })*//*
                .lineToLinearHeading(new Pose2d(12, -53, Math.toRadians(180)))
                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                *//*.addDisplacementMarker(() -> {
                    turnCommand.schedule();
                })*//*
                .lineToLinearHeading(new Pose2d(52, -53, Math.toRadians(180)))
                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                .build();

        *//*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //.lineTo(new Vector2d(-41, 52))
                .lineToLinearHeading(new Pose2d(56, -60, Math.toRadians(0)))
                .build();*//*

        *//*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //.lineToConstantHeading(new Vector2d(-37, 2))
                .lineToLinearHeading(new Pose2d(56, -60, Math.toRadians(0)))
                .build();*//*

        //LINE UP WITH CONE STACK
        *//*Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
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
                .build();*//*




        followPixel = new TrajectoryFollowerCommand(drive,pixelTraj);

        follower1 = new TrajectoryFollowerCommand(drive,traj1);
        follower2 = new TrajectoryFollowerCommand(drive,traj2);
        //follower3 = new TrajectoryFollowerCommand(drive,traj3);
        *//*follower4 = new TrajectoryFollowerCommand(drive,traj4);
        follower5 = new TrajectoryFollowerCommand(drive,traj5);
        follower6a = new TrajectoryFollowerCommand(drive,traj6a);
        follower6b = new TrajectoryFollowerCommand(drive,traj6b);
        follower7 = new TrajectoryFollowerCommand(drive,traj7);
        follower8 = new TrajectoryFollowerCommand(drive,traj8);
        follower9 = new TrajectoryFollowerCommand(drive,traj9);*/
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
        detectTEPositionCommand = createPositionIdentifierMechanism.getDetectTEPositionCommand();
        //holds Purple Pixels
        grabberCloseLeftCommand = createPixelGrabberMechanism.createGrabberLeftCloseCommand();
        grabberOpenLeftCommand = createPixelGrabberMechanism.createGrabberLeftOpenCommand();
        //Holds Yellow Pixel
        grabberOpenRightCommand = createPixelGrabberMechanism.createGrabberRightOpenCommand();

        grabberWristDropCommand = createGrabberWristMechanism.createGrabberWristDropCommand();
        grabberWristPickUpCommand = createGrabberWristMechanism.createGrabberWristPickUpCommand();

        turnCommand120 = new TurnCommand(drive, Math.toRadians(120));

        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(

               /* detectTEPositionCommand.andThen(new InstantCommand(()->{
                    telemetry.addData("Selection Position Stage Side Red", Positions.getInstance().getTEPosition());
                    telemetry.update();
                }))));*/

                grabberWristPickUpCommand,
                detectTEPositionCommand.andThen(new InstantCommand(()->{

                    telemetry.addData("Selection Position Stage Side Red", createPositionIdentifierMechanism.getPosition());
                    telemetry.update();

                    /*CreatePixelDropTrajectory createPixelDropTrajectory = new CreatePixelDropTrajectory(drive, createPositionIdentifierMechanism,startPose, telemetry);
                    Trajectory pixelTraj = createPixelDropTrajectory.createTrajectory();

                    if(createPositionIdentifierMechanism.getPosition() != Positions.TEPosition.POSITION_LEFT){
                        Trajectory traj1 = drive.trajectoryBuilder(pixelTraj.end())

                                .lineToLinearHeading(new Pose2d(12, -53, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                                .build();

                        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                                .lineToLinearHeading(new Pose2d(52, -53, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                                .build();


                        followPixel = new TrajectoryFollowerCommand(drive,pixelTraj);

                        follower1 = new TrajectoryFollowerCommand(drive,traj1);
                        follower2 = new TrajectoryFollowerCommand(drive,traj2);

                        new SequentialCommandGroup(followPixel, grabberOpenRightCommand,grabberWristDropCommand, follower1, follower2).schedule();
                    }
                    else{


                        Trajectory traj1 = drive.trajectoryBuilder(pixelTraj.end())

                                .lineToLinearHeading(new Pose2d(10,-34, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                                .build();

                        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                                .lineToLinearHeading(new Pose2d(12, -53, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                                .build();

                        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())

                                .lineToLinearHeading(new Pose2d(52, -53, Math.toRadians(180)))
                                //.lineToLinearHeading(new Pose2d(-36,-50, Math.toRadians(90)))
                                .build();


                        followPixel = new TrajectoryFollowerCommand(drive,pixelTraj);

                        follower1 = new TrajectoryFollowerCommand(drive,traj1);
                        follower2 = new TrajectoryFollowerCommand(drive,traj2);
                        follower3 = new TrajectoryFollowerCommand(drive, traj3);

                        new SequentialCommandGroup(followPixel, follower1, grabberOpenRightCommand,grabberWristDropCommand, follower2, follower3).schedule();
                    }*/



                }))));
                //grabberWristPickUpCommand, detectTEPositionCommand.andThen(followPixel, turnCommand120, grabberOpenRightCommand,grabberWristDropCommand, follower1, follower2)));
                //grabberWristPickUpCommand, followPixel, turnCommand120, grabberOpenRightCommand,grabberWristDropCommand, follower1, follower2));
    }

}
