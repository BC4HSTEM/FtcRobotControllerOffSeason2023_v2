package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class ArmTeleop extends OpMode {

    private PIDController controller;

    public static double p = 0.0019;
    public static double i = 0;
    public static double d = 0.0001;
    public static double f = 6;

    public static int target = 0;

    public static double motorDegrees = 360.0;
    public static double largeGear = 108;

    public static double smallGear = 30;

    public double gearRatio = largeGear / smallGear;

    public double ticksPerRotation = 1120;
    private final double ticks_in_degree = (gearRatio * ticksPerRotation) / 360;

    private DcMotorEx arm;

    @Override
    public void init() {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop(){
        controller.setPID(p,i,d);
        
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        arm.setPower(power);

        telemetry.addData("pos, ", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("power ", power);

        telemetry.update();

    }

}
