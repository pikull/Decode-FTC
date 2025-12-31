/*package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.controller.PIDFController;


@TeleOp (name = "SlidesTest")
public class SlidesTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS
        DcMotorEx slidesRight = hardwareMap.get(DcMotorEx.class,"slidesRight");
        DcMotorEx slidesLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");

        //REVERSE + INITIATE ENCODERS


        slidesLeft.setDirection(DcMotorEx.Direction.REVERSE);

        slidesLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slidesLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slidesRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        PIDFCoefficients coefficients = new PIDFCoefficients(0.004, 0, 0, 0);
        slidesLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, coefficients);
        slidesRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, coefficients);




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                slidesLeft.setTargetPosition(1000);
            }
        }

    }
}*/