package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Config
@TeleOp
public class MeasureVelocityTeleop extends OpMode {



    private DcMotorEx arm;

    public static double currentVelocity;
    public static double maxVelocity = 0.0;


    @Override
    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(1);

    }
    public void loop(){
        currentVelocity = arm.getVelocity();

        if (currentVelocity > maxVelocity) {
            maxVelocity = currentVelocity;
        }



        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("maximum velocity", maxVelocity);
        telemetry.update();


    }

}
