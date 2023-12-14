package org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {

    private PIDController controller;

    public static double p = 0.001;
    public static double i = 0.001;
    public static double d = 0;
    public static double f = 0.001;

    public static int target = 50;

    private final double ticks_in_degree =  1120 / 360.0;

    private DcMotorEx arm;
    private Telemetry telemetry;

    public ArmSubsystem(DcMotorEx armMotor){

        arm = armMotor;
        createController();
    }
    //9. Added another constructor that takes in Telemetry
    public ArmSubsystem(DcMotorEx armMotor, Telemetry telemetry, boolean useDB){

        arm = armMotor;
        if (useDB){
            this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        else{
            this.telemetry = telemetry;
        }

        createController();
    }



    private void createController(){

        controller = new PIDController(p,i,d);
    }

    public void setTargetPIDPosition(int t){
        controller.setPIDF(p,i,d,f);
        target = t;
        setPower();
    }

    public void setTargetPosition(int t){
        arm.setTargetPosition(t);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);

    }

    public void stopResetEncoder(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    //// set the inversion factor
    //m_motor.setInverted(true);
    //
    //// get the inversion factor
    //boolean isInverted = m_motor.getInverted();
    //
    //// set the zero power behavior to BRAKE
    //m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    public void setZeroPowerBehavoir(DcMotorEx.ZeroPowerBehavior behavior){
        arm.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotorEx.Direction direction){
        arm.setDirection(direction);
    }


    public void setMode(DcMotorEx.RunMode mode){
        arm.setMode(mode);
    }

    private void setPower(){
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;

        arm.setPower(power);

        //telemetry.addData("power, ", power);
        //telemetry.addData("pos, ", armPos);
        //telemetry.addData("target ", target);

        //telemetry.update();
    }


}
