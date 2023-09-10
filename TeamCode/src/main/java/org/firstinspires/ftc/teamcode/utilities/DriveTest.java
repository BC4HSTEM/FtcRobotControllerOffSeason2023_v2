/*
Max Velocity Test for PIDF Tuning
https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit
Get maxVelocity by running this OpMode

F = 32767 / maxVelocity
P = 0.1 * F
I = 0.1 * P
D = 0
positionP = 5.0

 */
package org.firstinspires.ftc.teamcode.utilities;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.CreateDriveTrainMechanism;


@Disabled
@TeleOp(name="Drive Test", group="Utilities")
public class DriveTest extends LinearOpMode {


    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        GamepadEx driver1 = new GamepadEx(gamepad1);
        CreateDriveTrainMechanism createDriveTrainMechanism = new CreateDriveTrainMechanism(hardwareMap, "drive", driver1, telemetry, true);


        //init loop
        while (! isStarted()) {
            telemetry.addData("Drive Test", "This test is best run on top of a box to test each wheel");

            telemetry.addData("encder frontLeft", createDriveTrainMechanism.getDriveSubsystem().getFl().getCurrentPosition());
            telemetry.addData("encoder frontRight", createDriveTrainMechanism.getDriveSubsystem().getFr().getCurrentPosition());
            telemetry.addData("encoder backRight", createDriveTrainMechanism.getDriveSubsystem().getBr().getCurrentPosition());
            telemetry.addData("encoder backLeft", createDriveTrainMechanism.getDriveSubsystem().getBl().getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        //Code to test if the motors are functioning as intended. Leave commented out if not testing.


        timer.reset();
        while (timer.time() < 5000 && opModeIsActive()){

            telemetry.addData("encder frontLeft", createDriveTrainMechanism.getDriveSubsystem().getFl().getCurrentPosition());
            telemetry.addData("encoder frontRight", createDriveTrainMechanism.getDriveSubsystem().getFr().getCurrentPosition());
            telemetry.addData("encoder backRight", createDriveTrainMechanism.getDriveSubsystem().getBr().getCurrentPosition());
            telemetry.addData("encoder backLeft", createDriveTrainMechanism.getDriveSubsystem().getBl().getCurrentPosition());
            telemetry.update();
        }

        sleep(1000);

        createDriveTrainMechanism.getDriveSubsystem().getFl().motor.setPower(0.5);
        telemetry.addData("motor", "front left");
        telemetry.update();
        sleep(5000);
        createDriveTrainMechanism.getDriveSubsystem().getFl().motor.setPower(0);

        createDriveTrainMechanism.getDriveSubsystem().getFr().motor.setPower(0.5);
        telemetry.addData("motor", "front right");
        telemetry.update();
        sleep(5000);
        createDriveTrainMechanism.getDriveSubsystem().getFr().motor.setPower(0);

        createDriveTrainMechanism.getDriveSubsystem().getBr().motor.setPower(0.5);
        telemetry.addData("motor","back right");
        telemetry.update();
        sleep(5000);
        createDriveTrainMechanism.getDriveSubsystem().getBr().motor.setPower(0);

        createDriveTrainMechanism.getDriveSubsystem().getBl().motor.setPower(0.5);
        telemetry.addData("motor","back left");
        telemetry.update();
        sleep(5000);
        createDriveTrainMechanism.getDriveSubsystem().getBl().motor.setPower(0);

        createDriveTrainMechanism.getDriveSubsystem().getFl().motor.setPower(0.5);
        createDriveTrainMechanism.getDriveSubsystem().getFr().motor.setPower(0.5);
        telemetry.addData("motor","front Right and Left");
        telemetry.update();
        sleep(5000);
        createDriveTrainMechanism.getDriveSubsystem().getFl().motor.setPower(0);
        createDriveTrainMechanism.getDriveSubsystem().getFr().motor.setPower(0);

        createDriveTrainMechanism.getDriveSubsystem().getBl().motor.setPower(0.5);
        createDriveTrainMechanism.getDriveSubsystem().getBr().motor.setPower(0.5);
        telemetry.addData("motor","back Right and Left");
        telemetry.update();
        sleep(5000);
        createDriveTrainMechanism.getDriveSubsystem().getBl().motor.setPower(0);
        createDriveTrainMechanism.getDriveSubsystem().getBr().motor.setPower(0);

    }
}
