/*
    Drive Train Motor Test
    Use this opMode to check the motors if they need to be replaced or if there are inconsistencies.
    If an encoder cable is loose, bad, or miswired, the robot will not function properly.

    Place robot on a box before starting test

    During Init, turn each wheel to check the encoder cable
    After Start watch the telemetry to ensure the correct wheel is turning

 */
package org.firstinspires.ftc.teamcode.utilities;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.CreateDriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.subsystems.DriveSubsystem;


@Autonomous(name="Drivetrain Wiring/Encoder Test", group="Utilities")
public class DriveWiringTest extends LinearOpMode {
    GamepadEx driver1 = new GamepadEx(gamepad1);
    GamepadEx driver2 = new GamepadEx(gamepad2);

    CreateDriveTrainMechanism createDriveTrain = new CreateDriveTrainMechanism(hardwareMap, "drive", driver1, telemetry, true);
    DriveSubsystem driveSubsystem = createDriveTrain.getDriveSubsystem();
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        //robot.init(hardwareMap, telemetry);

        //init loop
        while (! isStarted()) {
            telemetry.addData("Instructions", "Place robot on a box to check encoders during initialization and the motor wiring after start");
            telemetry.addData("Drivetrain Encoder Test", "Turn each wheel to check the encoder cable");
            telemetry.addData("encoder frontLeft", driveSubsystem.getFl().getCurrentPosition());
            telemetry.addData("encoder frontRight", driveSubsystem.getFr().getCurrentPosition());
            telemetry.addData("encoder backRight", driveSubsystem.getFr().getCurrentPosition());
            telemetry.addData("encoder backLeft", driveSubsystem.getBl().getCurrentPosition());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Motor Wiring Test", "Watch telemetry to ensure the correct wheel is moving based on the port it is mapped to.");

        sleep(5000);

        trackMotorAndEncoderTimeDelay((DcMotorEx) driveSubsystem.getFl(), "frontLeft");
        trackMotorAndEncoderTimeDelay((DcMotorEx) driveSubsystem.getFr(), "frontRight");
        trackMotorAndEncoderTimeDelay((DcMotorEx) driveSubsystem.getBr(), "backRight");
        trackMotorAndEncoderTimeDelay((DcMotorEx) driveSubsystem.getBl(), "backLeft");

        sleep(5000);

        driveSubsystem.getFl().motor.setPower(0.5);
        driveSubsystem.getFr().motor.setPower(0.5);
        telemetry.addData("motor","front Right and Left");
        telemetry.update();
        sleep(5000);
        driveSubsystem.getFl().motor.setPower(0);
        driveSubsystem.getFr().motor.setPower(0);

        driveSubsystem.getBl().motor.setPower(0.5);
        driveSubsystem.getBr().motor.setPower(0.5);
        telemetry.addData("motor","back Right and Left");
        telemetry.update();
        sleep(5000);
        driveSubsystem.getBl().motor.setPower(0);
        driveSubsystem.getBr().motor.setPower(0);

    }

    private void trackMotorAndEncoderTimeDelay(DcMotorEx motorToTrack, String motorConfigName){
        motorToTrack.setPower(0.5);
        timer.reset();
        while (timer.milliseconds() < 5000 && opModeIsActive()){
            telemetry.addData("motor", motorConfigName);
            telemetry.addData(motorConfigName, motorToTrack.getCurrentPosition());
            telemetry.update();
        }
        motorToTrack.setPower(0);
    }
}
