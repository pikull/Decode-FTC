package org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.roadrunner.HolonomicController;

import org.firstinspires.ftc.teamcode.controller.PIDController;
import org.firstinspires.ftc.teamcode.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp (name = "TeleopV2")
public class TeleopV2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor slidesRight = hardwareMap.dcMotor.get("slidesRight");
        DcMotor slidesLeft = hardwareMap.dcMotor.get("slidesLeft");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo rightExtendoServo = hardwareMap.servo.get("rightExtendoServo");
        Servo leftExtendoServo = hardwareMap.servo.get("leftExtendoServo");
        Servo intakeRight = hardwareMap.servo.get("intakeRight");
        Servo intakeLeft = hardwareMap.servo.get("intakeLeft");
        Servo intakeWrist = hardwareMap.servo.get("intakeWrist");

        Servo rightArm = hardwareMap.servo.get("rightArm");
        Servo leftArm = hardwareMap.servo.get("leftArm");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo clawServo = hardwareMap.servo.get("clawMotor");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class,"Limelight");

        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;
        PIDFController controller = new PIDFController(kp, ki, kd, kf);
        
        //REVERSE + INITIATE ENCODERS
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        slidesRight.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightExtendoServo.setDirection(Servo.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 1;
        double intakeSlowPower = 0;
        double intakeWrist_IntakingPos = 0.665;
        double intakeWrist_RetractedPos = 0.09;
        double extendoRetractedPos = 0.45  ;
        double extendoExtendedPos = 0.675;
        double fourBarRetractedPos = 0.3194;
        double fourBarExtendedPos = 0.9061; // maybe 0.75

        double armInitPos = 0.65;
        double armPickupPos = 0.6;
        double armDropPos = 0.35;
        double armVerticalPos = 0.5;
        double wristInitPos = 0.35;
        double wristPickupPos = 0.25 ;
        double wristVerticalPos = 0.5;
        double wristDropPos = 0.6494;
        double clawOpenPos = 0.55;
        double clawClosePos = 0.35;

        double groundLevel = 0;
        double midLevel = 1250;
        double highLevel = 2600;

        double startMovingArmBackDistFromTarget = 600;


        //INIATE MOTOR POSITIONS
        leftExtendoServo.setPosition(extendoRetractedPos);
        rightExtendoServo.setPosition(extendoRetractedPos);
        rightArm.setPosition(wristInitPos);
        leftArm.setPosition(wristInitPos);
        clawWrist.setPosition(armInitPos);
        clawServo.setPosition(clawOpenPos);

        intakeWrist.setPosition(intakeWrist_RetractedPos);
        intake.setPower(stopPower);
        intakeRight.setPosition(fourBarRetractedPos);
        intakeLeft.setPosition(fourBarRetractedPos);


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

            int slidesleftpos = slidesLeft.getCurrentPosition();
            double powerLeft = controller.calculate(slidesleftpos, targets);

            int slidesrightpos = slidesRight.getCurrentPosition();
            double powerRight = controller.calculate(slidesrightpos, targets);

            slidesLeft.setPower(powerLeft);
            slidesRight.setPower(powerRight);

            // SCISSOR LIFT + INTAKE ///////////////

            // extends scissor lift, extends 4bar
            if (gamepad1.right_bumper && !scissor_extended && gamepad1_rightBumperReleased) {
                intakeWrist.setPosition(intakeWrist_IntakingPos);
                rightExtendoServo.setPosition(extendoExtendedPos);
                leftExtendoServo.setPosition(extendoExtendedPos);
                intakeRight.setPosition(fourBarExtendedPos);
                intakeLeft.setPosition(fourBarExtendedPos);
                intake.setPower(intakePower);
                scissor_extended = true;
                intake_running = true;
                gamepad1_rightBumperReleased = false;
            }
            // inits scissor lift, extends 4bar
            if (gamepad1.left_bumper && !intake_running && gamepad1_leftBumperReleased) {
                intakeWrist.setPosition(intakeWrist_IntakingPos);
                rightExtendoServo.setPosition(extendoRetractedPos);
                leftExtendoServo.setPosition(extendoRetractedPos);
                intakeRight.setPosition(fourBarExtendedPos);
                intakeLeft.setPosition(fourBarExtendedPos);

                intake.setPower(intakePower);
                intake_running = true;
                scissor_extended = false;
                gamepad1_leftBumperReleased = false;
            }
            // inits scissor lift, inits 4bar
            if ((gamepad1.left_bumper && gamepad1_leftBumperReleased && intake_running) || (gamepad1.right_bumper && gamepad1_rightBumperReleased && scissor_extended)) {
                intakeRight.setPosition(fourBarRetractedPos);
                intakeLeft.setPosition(fourBarRetractedPos);
                rightExtendoServo.setPosition(extendoRetractedPos);
                leftExtendoServo.setPosition(extendoRetractedPos);
                intakeWrist.setPosition(intakeWrist_RetractedPos);
                intake.setPower(intakeSlowPower);
                intake_running = false;
                scissor_extended = false;
                gamepad1_rightBumperReleased = false;
                gamepad1_leftBumperReleased = false;
            }

            if (gamepad1.left_trigger != 0) {
                intake.setPower(-1);
                gamepad1_leftTriggerReleased = false;
            }
            else if (gamepad1.right_trigger != 0) {
                intake.setPower(1);
            }
            else if (intake_running) {
                intake.setPower(1);
            }
            else if (!intake_running) {
                intake.setPower(0);
            }


            // GAMEPAD 2 CONTROLS ////////////////////////////////////////
            //SLIDES PID
            if (gamepad2.left_bumper && gamepad2_leftBumperReleased && !atDropPos) {
                leftArm.setPosition(wristInitPos);
                rightArm.setPosition(wristInitPos);
                clawWrist.setPosition(armInitPos);
                clawServo.setPosition(clawOpenPos);
                clawClosed = false;
                targets = groundLevel;
                armWristInited = true;
                gamepad2_leftBumperReleased = false;
            }
            else if (gamepad2.left_bumper && gamepad2_leftBumperReleased && atDropPos) {
                leftArm.setPosition(wristInitPos);
                rightArm.setPosition(wristInitPos);
                clawWrist.setPosition(armInitPos);
                justMovedToInitArmTime = System.currentTimeMillis();
                justMovedToInitArm = true;
                clawServo.setPosition(clawOpenPos);
                clawClosed = false;
                atDropPos = false;
                gamepad2_leftBumperReleased = false;

            }
            if (justMovedToInitArm && System.currentTimeMillis() - justMovedToInitArmTime >= 150) {
                targets = groundLevel;
                justMovedToInitArm = false;
                armWristInited = true;

            }

            if (gamepad2.right_bumper && gamepad2_rightBumperReleased && armWristInited && (slidesLeft.getCurrentPosition() < 50 || slidesRight.getCurrentPosition() < 50)) {
                clawWrist.setPosition(armPickupPos);
                leftArm.setPosition(wristPickupPos);
                rightArm.setPosition(wristPickupPos);
                wristInPickupPosTime = System.currentTimeMillis();
                gamepad2_rightBumperReleased = false;
                armWristInited = false;
                wristInPickupPos = true;
            }
            if (wristInPickupPos && System.currentTimeMillis() - wristInPickupPosTime >= 400) {
                clawServo.setPosition(clawClosePos);
                clawClosedTime = System.currentTimeMillis();
                wristInPickupPos = false;
                clawJustClosedOnSample = true;
                clawClosed = true;
            }
            if (clawJustClosedOnSample && System.currentTimeMillis() - clawClosedTime >= 300) {
                leftArm.setPosition(wristVerticalPos);
                rightArm.setPosition(wristVerticalPos);
                clawWrist.setPosition(armVerticalPos);
                clawJustClosedOnSample = false;

            }


            // click y: if tag in view, turn to center tag
            if (gamepad2.y) {
                while (result.getTx() > 3) {
                    leftFront.setPower(-.1);
                    leftBack.setPower(-.1);
                    rightFront.setPower(.1);
                    rightBack.setPower(.1);
                }
                while (result.getTx() < 3) {
                    leftFront.setPower(.1);
                    leftBack.setPower(.1);
                    rightBack.setPower(-.1);
                    rightFront.setPower(-.1);
                }
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }


            if (gamepad2.x && gamepad2_xReleased && clawClosed) {
                targets = midLevel;
                slidesGoingToMid = true;
                gamepad2_xReleased = false;
            }
            else if (gamepad2.x && gamepad2_xReleased && !clawClosed) {
                targets = midLevel;
                gamepad2_xReleased = false;
            }

            if (gamepad2.y && gamepad2_yReleased && clawClosed) {
                targets = highLevel;
                slidesGoingtoHigh = true;
                gamepad2_yReleased = false;
            }
            else if (gamepad2.y && gamepad2_yReleased && !clawClosed) {
                targets = highLevel;
                gamepad2_yReleased = false;
            }

            if ((slidesGoingToMid || slidesGoingtoHigh) && (slidesLeft.getCurrentPosition() >= targets - startMovingArmBackDistFromTarget || slidesRight.getCurrentPosition() >= targets - startMovingArmBackDistFromTarget)) {
                leftArm.setPosition(wristDropPos);
                rightArm.setPosition(wristDropPos);
                clawWrist.setPosition(armDropPos);
                slidesGoingToMid = false;
                slidesGoingtoHigh = false;
                atDropPos = true;
            }
            if ((gamepad2.left_trigger != 0) && gamepad2_leftTriggerReleased) {
                clawServo.setPosition(clawOpenPos);
                clawClosed = false;
                gamepad2_leftTriggerReleased = false;
            }

            if (gamepad2.right_trigger != 0 && gamepad2_rightTriggerReleased){
                clawServo.setPosition(clawClosePos);
                clawClosed = true;
                gamepad2_rightTriggerReleased = false;
            }
            if (gamepad2.dpad_left && gamepad2_dPadLeftReleased) {
                double currWristPos = leftArm.getPosition();
                leftArm.setPosition(currWristPos - 0.05);
                rightArm.setPosition(currWristPos - 0.05);
                gamepad2_dPadLeftReleased = false;
            }
            if (gamepad2.dpad_right && gamepad2_dPadRightReleased) {
                double currWristPos = leftArm.getPosition();
                leftArm.setPosition(currWristPos + 0.05);
                rightArm.setPosition(currWristPos + 0.05);
                gamepad2_dPadRightReleased = false;
            }
            if (gamepad2.dpad_right && gamepad2_dPadRightReleased) {
                double currArmPos = clawWrist.getPosition();
                clawWrist.setPosition(currArmPos - 0.05);
                gamepad2_dPadRightReleased = false;
            }
            if (gamepad2.dpad_left && gamepad2_dPadLeftReleased) {
                double currArmPos = clawWrist.getPosition();
                clawWrist.setPosition(currArmPos + 0.05);
                gamepad2_dPadLeftReleased = false;
            }
            if(gamepad1.a){

                while(result.getTx()!=0)
                rightFront.setPower(-.1);
                rightBack.setPower(-.1);
                leftBack.setPower(.1);
                leftFront.setPower(.1);
            }





            // CHECK IF BUTTONS RELEASED
            if (!gamepad1.right_bumper) {
                gamepad1_rightBumperReleased = true;
            }
            if (!gamepad1.left_bumper) {
                gamepad1_leftBumperReleased = true;
            }
            if (!gamepad1.dpad_up) {
                gamepad1_dPadUpReleased = true;
            }
            if (!gamepad1.dpad_down) {
                gamepad1_dPadDownReleased = true;
            }
            if (gamepad1.left_trigger == 0) {
                gamepad1_leftTriggerReleased = true;
            }
            if (!gamepad1.dpad_right) {
                gamepad1_dPadRightReleased = true;
            }
            if (!gamepad1.dpad_left) {
                gamepad1_dPadLeftReleased = true;
            }

            //ARM MOTOR POSITION TESTING

            if (gamepad1.right_trigger == 0) {
                gamepad1_rightTriggerReleased = true;
            }

            if (!gamepad2.x) {
                gamepad2_xReleased = true;
            }
            if (!gamepad2.y) {
                gamepad2_yReleased = true;
            }
            if (!gamepad2.left_bumper) {
                gamepad2_leftBumperReleased = true;
            }
            if (!gamepad2.right_bumper) {
                gamepad2_rightBumperReleased = true;
            }
            if (gamepad2.right_trigger == 0) {
                gamepad2_rightTriggerReleased = true;
            }
            if (gamepad2.left_trigger == 0) {
                gamepad2_leftTriggerReleased = true;
            }
            if (!gamepad2.dpad_left) {
                gamepad2_dPadLeftReleased = true;
            }
            if (!gamepad2.dpad_right) {
                gamepad2_dPadRightReleased = true;
            }


        }
    }

}
