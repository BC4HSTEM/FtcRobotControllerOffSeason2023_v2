package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class ArmTeleop extends OpMode {

    private PIDController controller;

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 3360 / 360.0;

    private DcMotorEx arm;

    @Override
    public void init() {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }
    public void loop(){
        controller = new PIDController(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        arm.setPower(power);

        telemetry.addData("pos, ", armPos);
        telemetry.addData("target ", target);

        telemetry.update();

    }

}
