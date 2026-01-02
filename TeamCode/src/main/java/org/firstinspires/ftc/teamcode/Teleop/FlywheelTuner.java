package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class FlywheelTuner extends OpMode {

    // general
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double curTargetVelocity = highVelocity;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    // left
    public DcMotorEx flywheelMotorL;
    double leftF = 0;
    double leftP = 0;

    // right
    public DcMotorEx flywheelMotorR;
    double rightF = 0;
    double rightP = 0;

    @Override
    public void init() {

        DcMotor flywheelMotor = hardwareMap.get(DcMotorEx.class, "leftShooter");

    }

    @Override
    public void loop() {

    }

}