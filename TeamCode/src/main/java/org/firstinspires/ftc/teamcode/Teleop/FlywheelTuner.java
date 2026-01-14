package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FlywheelTuner extends OpMode {
Servo outake;
    // general
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    // left
    public DcMotorEx flywheelMotorL;
    double leftF = 0;
    double leftP = 0;
    double curTargetVelocityL = highVelocity;

    // right
    public DcMotorEx flywheelMotorR;
    double rightF = 0;
    double rightP = 0;
    double curTargetVelocityR = highVelocity;

    @Override
    public void init() {
        outake = hardwareMap.servo.get("outakeS");
        flywheelMotorL = hardwareMap.get(DcMotorEx.class, "leftShooter");
        flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorL.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(leftP, 0, 0, leftF)
        );
        telemetry.addLine("Left Init Complete");
        outake.setPosition(0.5 );


        flywheelMotorR = hardwareMap.get(DcMotorEx.class, "rightShooter");
        flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotorR.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(rightP, 0, 0, rightF)
        );
        telemetry.addLine("Right Init Complete");
    }

    @Override
    public void loop() {

        /* ================= LEFT ================= */

        if (gamepad1.left_bumper && gamepad1.atRest()) {
            curTargetVelocityL =
                    (curTargetVelocityL == highVelocity) ? lowVelocity : highVelocity;
        }

        if (gamepad1.left_trigger > 0.5) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_left && gamepad1.atRest()) {
            leftF -= stepSizes[stepIndex];
        }

        if (gamepad1.dpad_right && gamepad1.atRest()) {
            leftF += stepSizes[stepIndex];
        }

        if (gamepad1.dpad_up && gamepad1.atRest()) {
            leftP += stepSizes[stepIndex];
        }

        if (gamepad1.dpad_down && gamepad1.atRest()) {
            leftP -= stepSizes[stepIndex];
        }

        flywheelMotorL.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(leftP, 0, 0, leftF)
        );

        flywheelMotorL.setVelocity(curTargetVelocityL);

        double curVelocityL = flywheelMotorL.getVelocity();
        double errorL = curTargetVelocityL - curVelocityL;

        telemetry.addLine("Left Motor");
        telemetry.addData("Target Velocity", curTargetVelocityL);
        telemetry.addData("Current Velocity", "%.2f", curVelocityL);
        telemetry.addData("Error", "%.2f", errorL);
        telemetry.addData("P", "%.4f", leftP);
        telemetry.addData("F", "%.4f", leftF);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine("-------------------------------");

        /* ================= RIGHT ================= */

        if (gamepad1.right_bumper && gamepad1.atRest()) {
            curTargetVelocityR =
                    (curTargetVelocityR == highVelocity) ? lowVelocity : highVelocity;
        }

        if (gamepad1.x && gamepad1.atRest()) rightF -= stepSizes[stepIndex];
        if (gamepad1.b && gamepad1.atRest()) rightF += stepSizes[stepIndex];
        if (gamepad1.y && gamepad1.atRest()) rightP += stepSizes[stepIndex];
        if (gamepad1.a && gamepad1.atRest()) rightP -= stepSizes[stepIndex];

        flywheelMotorR.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(rightP, 0, 0, rightF)
        );

        flywheelMotorR.setVelocity(curTargetVelocityR);


        double curVelocityR = flywheelMotorR.getVelocity();
        double errorR = curTargetVelocityR - curVelocityR;

        if(gamepad2.right_bumper&&gamepad2.atRest()){
            outake.setPosition(outake.getPosition()+0.01);
        }
        if(gamepad2.left_bumper&&gamepad1.atRest()){
            outake.setPosition(outake.getPosition()-0.01);
        }
        telemetry.addLine("Right Motor");
        telemetry.addData("Target Velocity", curTargetVelocityR);
        telemetry.addData("Current Velocity", "%.2f", curVelocityR);
        telemetry.addData("Error", "%.2f", errorR);
        telemetry.addData("P", "%.4f", rightP);
        telemetry.addData("F", "%.4f", rightF);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addData("servo",outake.getPosition());

        telemetry.update();
    }
}