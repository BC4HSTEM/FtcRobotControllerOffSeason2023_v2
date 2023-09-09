package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class HelloWorld extends OpMode {
    DcMotor LiftMotor = null;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    public void init(){
        LiftMotor = hardwareMap.get(DcMotor.class,"LM");
        FrontLeft = hardwareMap.get(DcMotor.class,"FL");
        FrontRight = hardwareMap.get(DcMotor.class,"FR");
        BackLeft = hardwareMap.get(DcMotor.class,"BL");
        BackRight = hardwareMap.get(DcMotor.class,"BR");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection((DcMotorSimple.Direction.REVERSE));
    }

    public void loop(){
        if (gamepad1.left_stick_x != 0.0 || gamepad1.left_stick_y != 0.0 ){
            FrontLeft.setPower(gamepad1.left_stick_x - gamepad1.left_stick_y);
            BackRight.setPower(gamepad1.left_stick_x - gamepad1.left_stick_y);
            BackLeft.setPower(-gamepad1.left_stick_x - gamepad1.left_stick_y);
            FrontRight.setPower(-gamepad1.left_stick_x - gamepad1.left_stick_y);
        }
        else{
            FrontLeft.setPower(0.0);
            BackRight.setPower(0.0);
            BackLeft.setPower(0.0);
            FrontRight.setPower(0.0);
        }

        if (gamepad1.right_stick_y != 0.0){
            FrontRight.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);
        }
        else{
            FrontRight.setPower(0.0);
            BackRight.setPower(0.0);
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
    }
}
