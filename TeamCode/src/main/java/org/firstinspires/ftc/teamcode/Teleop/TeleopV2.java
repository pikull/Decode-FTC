package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


// test


import org.firstinspires.ftc.teamcode.controller.PIDFController;



@TeleOp (name = "TeleopV2")
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
        CRServo outakeS = hardwareMap.get(CRServo.class,"outakeS");
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





        double startMovingArmBackDistFromTarget = 600;


        //INIATE MOTOR POSITIONS

        // CURRENT STATES
        boolean going_down = false;
        boolean scissor_extended = false;
        boolean intake_reversed = false;
        boolean intake_running = false;
        boolean extended_to_basket = false;
        boolean clawReset = true;
        boolean armWristInited = true;
        boolean wristInPickupPos = false;
        boolean clawJustClosedOnSample = false;
        boolean clawClosed = false;
        boolean slidesGoingToMid = false;
        boolean slidesGoingtoHigh = false;
        boolean atDropPos = false;
        boolean justMovedToInitArm = false;

        // BUTTON RELEASES
        boolean gamepad1_rightBumperReleased = true;
        boolean gamepad1_rightTriggerReleased = true;
        boolean gamepad1_leftBumperReleased = true;
        boolean gamepad1_dPadDownReleased = true;
        boolean gamepad1_dPadUpReleased = true;
        boolean gamepad1_dPadRightReleased = true;
        boolean gamepad1_dPadLeftReleased = true;
        boolean gamepad1_leftTriggerReleased = true;


        boolean gamepad2_xReleased = true;
        boolean gamepad2_yReleased = true;
        boolean gamepad2_leftBumperReleased = true;
        boolean gamepad2_rightBumperReleased = true;
        boolean gamepad2_rightTriggerReleased = true;
        boolean gamepad2_leftTriggerReleased = true;
        boolean gamepad2_dPadLeftReleased = true;
        boolean gamepad2_dPadRightReleased = true;
        boolean iPower1 = false;
// TIMES
        double wristInPickupPosTime = 0;
        double clawClosedTime = 0;
        double justMovedToDropPosTime = 0;
        double justMovedToInitArmTime = 0;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad2.a&& !iPower1){
                intake.setPower(1);
                iPower1 = true;

            }
            if(gamepad2.right_bumper&& iPower1){
                intake.setPower(0);
                iPower1 = false;
            }
            if (gamepad2.left_bumper){
                intake.setPower(-1);
            }


            // GAMEPAD 1 CONTROLS


            //ARM MOTOR POSITION TESTING


            //MECANUM DRIVE + SLIDES PIDF
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x ;

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
