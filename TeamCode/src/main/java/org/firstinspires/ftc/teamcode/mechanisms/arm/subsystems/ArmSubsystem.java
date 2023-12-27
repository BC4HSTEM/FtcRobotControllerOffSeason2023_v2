package org.firstinspires.ftc.teamcode.mechanisms.arm.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem extends SubsystemBase {

    private PIDController controller;

    public static double p = 0.001;
    public static double i = 0.0;
    public static double d = 0.0001;
    public static double f = 2;

    //public static int target = 0;

    public static double motorDegrees = 360.0;
    public static double largeGear = 108;

    public static double smallGear = 30;

    public double gearRatio = largeGear / smallGear;

    public double ticksPerRotation = 1120;

    public static int dropTargetPosition = 700;

    public static int dropMidPosition = 400;

    public static int pickUpTargetPosition = 20;
    private final double ticks_in_degree = (gearRatio * ticksPerRotation)  / motorDegrees;

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

    public void setDropTargetPIDPosition(){
        controller.setPID(p,i,d);
        setPower(dropTargetPosition);
    }

    public void setPickUpTargetPIDPosition(){
        controller.setPID(p,i,d);
        setPower(pickUpTargetPosition);
    }

    public void setMidDropTargetPIDPosition(){
        controller.setPID(p,i,d);
        setPower(dropMidPosition);
    }

    public void setTargetPosition(int t){
        arm.setTargetPosition(t);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);

    }

    public void setDropTargetPosition(){
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(dropTargetPosition);


    }

    public void setPickUpTargetPosition(){
        setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(pickUpTargetPosition);


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
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior){
        arm.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotorEx.Direction direction){

        arm.setDirection(direction);
    }


    public void setMode(DcMotorEx.RunMode mode){

        arm.setMode(mode);
    }

    public void stopArm(){
        arm.setPower(0);
    }

    public double getPowwr(){

        return arm.getPower();
    }

    public boolean isAtDropTargetPosition(){
        return arm.getTargetPosition() == dropTargetPosition;
    }

    public boolean isAtPickUpTargetPosition(){
        return arm.getTargetPosition() == pickUpTargetPosition;
    }

    public double getPosition(){
        return arm.getCurrentPosition();
    }

    public double getTarget(){

        return arm.getTargetPosition();
    }

    private void setPower(int target){
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid * ff;


        arm.setPower(power);




        telemetry.addData("power, ", power);
        telemetry.addData("pos, ", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("motor power ", arm.getPower());

        telemetry.update();


    }




}
