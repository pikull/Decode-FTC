package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.controller.PIDFController;

@TeleOp(name = "goon")
public class goon extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotorEx rightS = hardwareMap.get(DcMotorEx.class, "rightShooter");
        DcMotorEx leftS = hardwareMap.get(DcMotorEx.class, "leftShooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo outakeS = hardwareMap.servo.get("outakeS");
        CRServo intakeS = hardwareMap.get(CRServo.class, "intakeS");



        //REVERSE + INITIATE ENCODERS
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //POWERS & POSITIONS //////////////////////////////////
        intake.setPower(0);

        // LIMELIGHT
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        LLResult result = limelight.getLatestResult();


        // INITIATE MOTOR POSITIONS
        // CURRENT STATES
        boolean iPower1 = false;
        double ta = 0;
        int farshot = 1500;
        int closeshot = 1100;
        double outakePos = 0.2;
        double outakeFarPos = 0.1;

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp:%1fC, CPU:%.1f%%,FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
            result = limelight.getLatestResult();
            if (result.isValid()) {
                double tx = result.getTx();
                telemetry.addData("result", tx);
                telemetry.update();
            }
            // GAMEPAD 2 CONTROLS

            if (gamepad2.right_bumper && !iPower1) {
                intake.setPower(1);
                iPower1 = true;

            }
            if (gamepad2.right_bumper && iPower1) {
                intake.setPower(0);
                iPower1 = false;
            }
            if (gamepad2.left_bumper) {
                intake.setPower(-1);
                intake.setPower(-1);
                leftS.getPower();
            }

            // rotato potato until see april tag
            // click y: if tag in view, turn to center tag
            if (gamepad2.y) {
                double goon = 0.0;
                if (result.isValid()) {
                    goon = result.getTx();
                }
                while (result.isValid() && goon > 1) {
                    leftFront.setPower(.15);
                    leftBack.setPower(.15);
                    rightFront.setPower(-.15);
                    rightBack.setPower(-.15);
                    telemetry.addData("result", goon);
                    telemetry.update();
                    result = limelight.getLatestResult();
                    goon = result.getTx();
                }
                while (result.isValid() && result.getTx() < 1) {
                    leftFront.setPower(-.15);
                    leftBack.setPower(-.15);
                    rightBack.setPower(.15);
                    rightFront.setPower(.15);
                    telemetry.addData("result", goon);
                    telemetry.update();
                    result = limelight.getLatestResult();
                    goon = result.getTx();
                }
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
            if (gamepad2.x) {
                if (result.isValid()) {
                    ta = result.getTa();
                }
                if (calcDistance.getDistance(ta) < 80) {
                    rightS.setVelocity(closeshot);
                    leftS.setVelocity(closeshot);
                    outakeS.setPosition(outakePos);
                    intakeS.setPower(1);
                } else {
                    rightS.setVelocity(farshot);
                    leftS.setVelocity(farshot);
                    outakeS.setPosition(outakeFarPos);
                    intakeS.setPower(1);

                }
            }

            // GAMEPAD 1 CONTROLS
            //ARM MOTOR POSITION TESTING
            //MECANUM DRIVE + SLIDES PIDF
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

        }



    }

            // SCISSOR LIFT + INTAKE ///////////////
            // extends scissor lift, extends 4bar
}



