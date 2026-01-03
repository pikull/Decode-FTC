package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuner extends OpMode {

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

        flywheelMotorL = hardwareMap.get(DcMotorEx.class, "leftShooter");
        flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorL.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(leftP, 0, 0, leftF)
        );
        telemetry.addLine("Left Init Complete");

        flywheelMotorR = hardwareMap.get(DcMotorEx.class, "rightShooter");
        flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotorR.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(rightP, 0, 0, rightF)
        );
        telemetry.addLine("Right Init Complete");
    }

    @Override
    public void loop() {

        /* ================= LEFT ================= */

        if (gamepad1.left_bumper) {
            curTargetVelocityL =
                    (curTargetVelocityL == highVelocity) ? lowVelocity : highVelocity;
        }

        if (gamepad1.left_trigger > 0.5) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpad_left) {
            leftF -= stepSizes[stepIndex];
        }

        if (gamepad1.dpad_right) {
            leftF += stepSizes[stepIndex];
        }

        if (gamepad1.dpad_up) {
            leftP += stepSizes[stepIndex];
        }

        if (gamepad1.dpad_down) {
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

        if (gamepad1.right_bumper) {
            curTargetVelocityR =
                    (curTargetVelocityR == highVelocity) ? lowVelocity : highVelocity;
        }

        if (gamepad1.x) rightF -= stepSizes[stepIndex];
        if (gamepad1.b) rightF += stepSizes[stepIndex];
        if (gamepad1.y) rightP += stepSizes[stepIndex];
        if (gamepad1.a) rightP -= stepSizes[stepIndex];

        flywheelMotorR.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(rightP, 0, 0, rightF)
        );

        flywheelMotorR.setVelocity(curTargetVelocityR);

        double curVelocityR = flywheelMotorR.getVelocity();
        double errorR = curTargetVelocityR - curVelocityR;

        telemetry.addLine("Right Motor");
        telemetry.addData("Target Velocity", curTargetVelocityR);
        telemetry.addData("Current Velocity", "%.2f", curVelocityR);
        telemetry.addData("Error", "%.2f", errorR);
        telemetry.addData("P", "%.4f", rightP);
        telemetry.addData("F", "%.4f", rightF);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);

        telemetry.update();
    }
}