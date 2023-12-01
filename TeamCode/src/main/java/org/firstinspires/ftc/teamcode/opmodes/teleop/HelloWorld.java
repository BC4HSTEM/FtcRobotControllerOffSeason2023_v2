package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class HelloWorld extends OpMode {
    DcMotor LiftMotor = null;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotorEx Arm = null;


    public void init(){
        LiftMotor = hardwareMap.get(DcMotor.class,"LM");
        FrontLeft = hardwareMap.get(DcMotor.class,"FL");
        FrontRight = hardwareMap.get(DcMotor.class,"FR");
        BackLeft = hardwareMap.get(DcMotor.class,"BL");
        BackRight = hardwareMap.get(DcMotor.class,"BR");

        // Sydney Added the arm!
        Arm = hardwareMap.get(DcMotorEx.class,"arm" );
        Arm.setTargetPosition(0);
        // Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Arm.setPower(1.0);

        // THE RIGHT motors needed to be reversed
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection((DcMotorSimple.Direction.REVERSE));
        telemetry.addData("app started","hooray");
        telemetry.addData("arm","init");
    }

    public void loop(){

        if (gamepad1.left_stick_x != 0.0 || gamepad1.left_stick_y != 0.0 || gamepad1.right_stick_x != 0 ){
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            FrontLeft.setPower(y + x + rx);
            BackLeft.setPower(y - x + rx);
            FrontRight.setPower(y - x - rx);
            BackRight.setPower(y + x - rx);
        }
        else {
            FrontLeft.setPower(0.0);
            BackRight.setPower(0.0);
            BackLeft.setPower(0.0);
            FrontRight.setPower(0.0);
        }
        if(gamepad1.left_trigger != 0.0){
            LiftMotor.setPower(gamepad1.left_trigger);

        }
        else{
            LiftMotor.setPower(0.0);
        }
        if(gamepad1.right_trigger != 0.0){
            LiftMotor.setPower(-gamepad1.right_trigger);

        }
        else{
            LiftMotor.setPower(0.0);
        }


        if(gamepad1.right_bumper ){
            Arm.setTargetPosition(600);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }else if(gamepad1.left_bumper ){
            Arm.setTargetPosition(0);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        telemetry.addData("arm", Arm.getCurrentPosition());
        telemetry.update();



    }
}
