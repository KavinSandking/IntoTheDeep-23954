package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class lift_encoder extends LinearOpMode {
    DcMotorEx Slides;
    DcMotorEx Slides2;
    @Override
    public void runOpMode(){

        Slides = hardwareMap.get(DcMotorEx.class,"Slides");
        Slides2 = hardwareMap.get(DcMotorEx.class,"Slides2");
        Slides.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad2.right_stick_y==1.0){
                Slides.setPower(1.0);
                Slides2.setPower(1);
            }
            else if (gamepad2.right_stick_y==-1.0){
                Slides.setPower(-1);
                Slides2.setPower(-1);
            } else if (gamepad2.right_stick_y == 0.0){
                Slides.setPower(0);
                Slides2.setPower(0);

            }
            telemetry.addData("Slides",Slides.getCurrentPosition());
            telemetry.addData("Slides2",Slides2.getCurrentPosition());
            telemetry.update();
        }

    }

}