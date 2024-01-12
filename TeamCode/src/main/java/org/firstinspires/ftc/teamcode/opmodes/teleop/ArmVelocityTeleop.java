package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Config
@TeleOp
public class ArmVelocityTeleop extends OpMode {

    private PIDFController controller;

    public static int target = 0;

    //current velocity: 3200.0
    //maximum velocity: 3260.0
    private static double maximumVelocity = 3260.0;
    public static double f = 32767 / maximumVelocity; //10.051227
    public static double p = 0.0000000000000000000000000000001 * f;
    //public static double p = 0.6 * f;
    public static double i = 0.05 * p;
    //public static double i = 0.00 * p;
    public static double d = 0.0;

    public static double motorDegrees = 360.0;
    public static double largeGear = 108;

    public static double smallGear = 30;

    public double gearRatio = largeGear / smallGear;

    public double ticksPerRotation = 1120;
    private final double ticks_in_degree = (gearRatio * ticksPerRotation) / 360;

    private MotorEx arm;

    @Override
    public void init() {

        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        arm = new MotorEx(hardwareMap, "arm");
        //arm.setVelocityPIDFCoefficients(p,i,d,f);
        arm.resetEncoder();
        arm.setRunMode(Motor.RunMode.VelocityControl);
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void loop(){

        controller.setSetPoint(target);
        controller.setPIDF(p,i,d,f);

        if (!controller.atSetPoint()) {
            double pidf = controller.calculate(
                    arm.getCurrentPosition()  // the measured value
            );
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double velocity =  pidf * ff;
            arm.setVelocity(pidf);


        }
        else{
            arm.stopMotor();
        }

        telemetry.addData("pos, ", arm.getCurrentPosition());
        telemetry.addData("target ", target);
        telemetry.addData("at setppint ", controller.atSetPoint());
        telemetry.addData("velocity ", arm.getVelocity());

        telemetry.update();




    }

}
