package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.globals.Alliance;

import org.firstinspires.ftc.teamcode.globals.Path;
import org.firstinspires.ftc.teamcode.globals.Positions;
import org.firstinspires.ftc.teamcode.globals.Side;
//import org.firstinspires.ftc.teamcode.mechanisms.leds.CreateLEDMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.position_identifier.CreatePositionIdentifierMechanism;

//import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BlueAllianceStageSidePath1;
//import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BlueAllianceNonStageSidePath1;
//import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.RedAllianceStageSidePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BlueAllianceNonStageSidePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.RedAllianceNonStageSidePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.BlueAllianceStageSidePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.RedAllianceStageSidePath1;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.RedAllianceStageSidePath2;

@Autonomous(name="Auto Gamepad", group="Stage")
public class AutonomousGamePad extends CommandOpMode {
    //final String[] selectedAlliance = new String[1];

    //final String[] selectedSide = new String[1];
    //final String[] selectedPath = new String[1];


    final Pose2d blueAllianceNonStageSideStartPose = new Pose2d(-36, 60, Math.toRadians(270));
    final Pose2d blueAllianceStageSideStartPose = new Pose2d(12,60, Math.toRadians(270));
    final Pose2d redAllianceNonStageSideStartPose = new Pose2d(-36, -60, Math.toRadians(90));
    final Pose2d redAllianceStageSideStartPose = new Pose2d(12,-60, Math.toRadians(90));

    private Pose2d selectedStartPos = new Pose2d(0,0);

    private SequentialCommandGroup webCamGroup;

    CreatePositionIdentifierMechanism createPositionIdentifierMechanism;

    //private CreateLEDMechanism createLEDs;


    private final Vector2d[] finalPosition = new Vector2d[1];

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry.setAutoClear(false);
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //selectedAlliance[0] = "None";
        //selectedSide[0] = "None";
        //selectedPath[0] = "None";


        //Must reset static Positions global

        createPositionIdentifierMechanism = new CreatePositionIdentifierMechanism(hardwareMap, "Webcam 1", telemetry);
        createPositionIdentifierMechanism.createAuto();

        GamepadEx driver1 = new GamepadEx(gamepad1);

        showOptions(true);

        configureAllianceButtons(driver1);
        configureSelectSideButtons(driver1);
        configureSelectPathButtons(driver1);
        configureExecutePath(driver1);


        //createLEDs = new CreateLEDMechanism(hardwareMap, "blinkin");

        //determineTEPosition();




        while (!this.isStarted()){

            CommandScheduler.getInstance().run();
        }
    }

    private void determineTEPosition(){

        createPositionIdentifierMechanism.getDetectTEPositionCommand().andThen(new InstantCommand(this::showTSEPosition)).schedule();

    }

    private void configureAllianceButtons(GamepadEx op){

        op.getGamepadButton(GamepadKeys.Button.X).whenPressed(()->{
            //selectedAlliance[0] = "Blue";
            Alliance.getInstance().setAllianceTeam(Alliance.AllianceTeam.BLUE);
            //createLEDs.createAuto();
            showSelection();
        });

        op.getGamepadButton(GamepadKeys.Button.B).whenPressed(()->{
            //selectedAlliance[0] = "Red";
            Alliance.getInstance().setAllianceTeam(Alliance.AllianceTeam.RED);
            //createLEDs.createAuto();
            showSelection();
        });


    }

    private void configureSelectSideButtons(GamepadEx op){



        InstantCommand yBind = new InstantCommand(()->{
            //selectedSide[0] = "Blue";
            Side.getInstance().setPositionSide(Side.PositionSide.STAGE_SIDE);
            showSelection();

        });

        InstantCommand aBind = new InstantCommand(()->{
            //selectedSide[0] = "Red";
            Side.getInstance().setPositionSide(Side.PositionSide.NON_STAGE_SIDE);
            showSelection();

        });

        op.getGamepadButton(GamepadKeys.Button.Y).whenPressed(yBind).whenReleased(yBind::cancel);

        op.getGamepadButton(GamepadKeys.Button.A).whenPressed(aBind).whenReleased(aBind::cancel);


    }

    private void configureSelectPathButtons(GamepadEx op){


        op.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(()->{
            //selectedPath[0] = "Path 1";
            Path.getInstance().setSelectedPathToFollow(Path.PositionToFollow.PATH_1);
            showSelection();
        });

        op.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(()->{
            //selectedPath[0] = "Path 2";
            Path.getInstance().setSelectedPathToFollow(Path.PositionToFollow.PATH_2);
            showSelection();
        });

        op.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()->{
            //selectedPath[0] = "Path 3";
            Path.getInstance().setSelectedPathToFollow(Path.PositionToFollow.PATH_3);
            showSelection();
        });

        op.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(()->{
            //selectedPath[0] = "Path 4";
            Path.getInstance().setSelectedPathToFollow(Path.PositionToFollow.PATH_4);
            showSelection();
        });
    }

    private void configureExecutePath(GamepadEx op){
        op.getGamepadButton(GamepadKeys.Button.START).whenPressed(()->{


            telemetry.addLine("Path Executing.....");


            //determineTEPosition();
            //showSelection();


            executePath();

            //createPositionIdentifierMechanism.getStopDetectTEPosition().schedule();
            //createPositionIdentifierMechanism.getDetectTEPositionCommand().whenFinished(this::executePath).schedule();


        });

    }

    private void executePath(){


        if (Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){
            if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                selectedStartPos = blueAllianceStageSideStartPose;
            } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                selectedStartPos = redAllianceStageSideStartPose;
            }
        }
        else if(Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
            if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE) {
                selectedStartPos = blueAllianceNonStageSideStartPose;
            } else if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED) {
                selectedStartPos = redAllianceNonStageSideStartPose;
            }
        }

        //telemetry.clearAll();
        telemetry.addData("Selections Complete", String.format("Alliance: %s - Side: %s - Path: %s - Start Position: %s - TGE Side: %s",
                Alliance.getInstance().getAllianceTeam().toString(),
                Side.getInstance().getPositionSide().toString(),
                Path.getInstance().getSelectedPathToFollow(),selectedStartPos, "here"));
        telemetry.update();



        if (Path.getInstance().getSelectedPathToFollow() == Path.PositionToFollow.PATH_1) {
            Path.getInstance().setSelectedPathToFollow(Path.PositionToFollow.PATH_1);
            if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE) {
                BlueAllianceStageSidePath1 bsPath1 = new BlueAllianceStageSidePath1(hardwareMap, createPositionIdentifierMechanism,selectedStartPos, telemetry);
                bsPath1.createPath();
                bsPath1.execute(this);

            } else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){


                RedAllianceStageSidePath1 rsPath1 = new RedAllianceStageSidePath1(hardwareMap, createPositionIdentifierMechanism,selectedStartPos, telemetry);
                rsPath1.createPath();
                rsPath1.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
                BlueAllianceNonStageSidePath1 bnsPath1 = new BlueAllianceNonStageSidePath1(hardwareMap, createPositionIdentifierMechanism,selectedStartPos, telemetry);
                bnsPath1.createPath();
                bnsPath1.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
                RedAllianceNonStageSidePath1 rnsPath1 = new RedAllianceNonStageSidePath1 (hardwareMap, selectedStartPos, telemetry);
                rnsPath1.createPath();
                rnsPath1.execute(this);
            }
        }

        if (Path.getInstance().getSelectedPathToFollow() == Path.PositionToFollow.PATH_2) {
            Path.getInstance().setSelectedPathToFollow(Path.PositionToFollow.PATH_2);
            if (Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE) {
                //BlueAllianceStageSidePath2 bsPath2 = new BlueAllianceStageSidePath2(hardwareMap, selectedStartPos, telemetry);
                //bsPath2.createPath();
                //bsPath2.execute(this);

            } else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.STAGE_SIDE){

                RedAllianceStageSidePath2 rsPath2 = new RedAllianceStageSidePath2(hardwareMap, createPositionIdentifierMechanism,selectedStartPos, telemetry);
                rsPath2.createPath();
                rsPath2.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.BLUE && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
                //BlueAllianceNonStageSidePath2 bnsPath2 = new BlueAllianceNonStageSidePath2(hardwareMap, createPositionIdentifierMechanism,selectedStartPos, telemetry);
                //bnsPath2.createPath();
                //bnsPath2.execute(this);
            }
            else if(Alliance.getInstance().getAllianceTeam() == Alliance.AllianceTeam.RED && Side.getInstance().getPositionSide() == Side.PositionSide.NON_STAGE_SIDE){
                //RedAllianceNonStageSidePath2 rnsPath2 = new RedAllianceNonStageSidePath2 (hardwareMap, createPositionIdentifierMechanism,selectedStartPos, telemetry);
                //rnsPath2.createPath();
                //rnsPath2.execute(this);
            }
        }

        if (Path.getInstance().getSelectedPathToFollow() == Path.PositionToFollow.PATH_3) {}

        if (Path.getInstance().getSelectedPathToFollow() == Path.PositionToFollow.PATH_4) {}
    }

    private void showOptions(boolean update){

        telemetry.addLine("Press (X) for BLUE Alliance, (B) for RED Alliance");
        telemetry.addLine("Press (Y) for STAGE Side, (A) for NON STAGE Side");
        telemetry.addLine("Press (^) for Path 1");
        telemetry.addLine("Press (>) for Path 2");
        telemetry.addLine("Press (v) for Path 3");
        telemetry.addLine("Press (<) for Path 4");


        if(update){
            showTSEPosition();
            telemetry.update();
        }
    }

    private void showSelection(){
        showOptions(false);
        telemetry.addData("Selections ", String.format("Alliance: %s - Side: %s - Path: %s - Position: %s",Alliance.getInstance().getAllianceTeam().toString(),
                Side.getInstance().getPositionSide().toString(),
                Path.getInstance().getSelectedPathToFollow(),selectedStartPos));

        showTSEPosition();
        telemetry.update();

    }

    private void showTSEPosition(){
       telemetry.addData("Tag snapshot id in Autono:", createPositionIdentifierMechanism.getPosition());
        telemetry.addLine("\n");
    }



}
