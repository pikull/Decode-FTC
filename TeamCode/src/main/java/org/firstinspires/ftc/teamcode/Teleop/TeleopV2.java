package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.hardware.limelightvision.LLResult;


import org.firstinspires.ftc.teamcode.controller.PIDFController;


@TeleOp(name = "TeleopV2")
public class TeleopV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor rightS = hardwareMap.dcMotor.get("rightShooter");
        DcMotor leftS = hardwareMap.dcMotor.get("leftShooter");
        DcMotor intake = hardwareMap.dcMotor.get("intake");


        Servo intakeS = hardwareMap.servo.get("intakeS");
        CRServo outakeS = hardwareMap.get(CRServo.class, "outakeS");
        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;

        PIDFController controller = new PIDFController(kp, ki, kd, kf);

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


        double startMovingArmBackDistFromTarget = 600;


        //INIATE MOTOR POSITIONS

        // CURRENT STATES
        boolean iPower1 = false;


        waitForStart();

        if (isStopRequested()) return;

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

            if (gamepad2.a && !iPower1) {
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
            // then change angle based on distance
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
                while (result.isValid() && goon < 1) {
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


            // SCISSOR LIFT + INTAKE ///////////////

            // extends scissor lift, extends 4bar


        }
    }

}
