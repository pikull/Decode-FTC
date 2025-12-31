package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.PIDFController;


public class DrivingClass extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;
        PIDFController controller = new PIDFController(kp, ki, kd, kf);


        //REVERSE + INITIATE ENCODERS
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);


        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 0.71;
        double intakeSlowPower = 0.15;
        double intakeAngle_IntakingPos = 0.54;
        double intakeAngle_RetractedPos = 0.07;
        double extendoRetractedPos = 0.43  ;
        double extendoExtendedPos = 0.6;
        double fourBarRetractedPos = 0.15;
        double fourBarExtendedPos = 0.7917; // maybe 0.75

        double armInitPos = 0.6;
        double armPickupPos = 0.45;
        double armDropPos = 0.85;
        double armVerticalPos = 0.75;
        double wristInitPos = 0.8;
        double wristPickupPos = 0.9;
        double wristVerticalPos = 0.45;
        double wristDropPos = 0.0;
        double clawOpenPos = 0.4;
        double clawClosePos = 0.6;

        double groundLevel = 0;
        double initLevel = 100;
        double midLevel = 1250;
        double highLevel = 2750;

        double startMovingArmBackDistFromTarget = 600;


        //INIATE MOTOR POSITIONS


        double targets = groundLevel;

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
        boolean gamepad2_dPadDownReleased = true;
        boolean gamepad2_dPadUpReleased = true;
// TIMES
        double wristInPickupPosTime = 0;
        double clawClosedTime = 0;
        double justMovedToDropPosTime = 0;
        double justMovedToInitArmTime = 0;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // GAMEPAD 1 CONTROLS


            //ARM MOTOR POSITION TESTING


            //MECANUM DRIVE + SLIDES PIDF
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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

}
