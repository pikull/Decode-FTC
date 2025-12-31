/*package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.PIDFController;


@TeleOp (name = "TeleopTuning")
public class TeleopTuning extends LinearOpMode {
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

        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;
        PIDFController controller = new PIDFController(kp, ki, kd, kf);


        //REVERSE + INITIATE ENCODERS
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightExtendoServo.setDirection(Servo.Direction.REVERSE);
        slidesRight.setDirection(DcMotor.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 1;
        double intakeSlowPower = 0;
        double intakeWrist_IntakingPos = 0.665;
        double intakeWrist_RetractedPos = 0.05;
        double extendoRetractedPos = 0.45  ;
        double extendoExtendedPos = 0.7;
        double fourBarRetractedPos = 0.3194;
        double fourBarExtendedPos = 0.9061;

        double armInitPos = 0.5;
        double armPickupPos = 0.6;
        double armDropPos = 0.5;
        double leftArmInit = 0.5;
        double rightArmInit = 0.5;
        double wristPickupPos = 0.25;
        double wristDropPos = 0.5;
        double wristInitPos = 0.5;
        double clawOpenPos = 0.55;
        double clawClosePos = 0.385;

        double groundLevel = 0;
        double initLevel = 100;
        double midLevel = 1250;
        double highLevel = 2600;


        //INIATE MOTOR POSITIONS
        leftExtendoServo.setPosition(extendoRetractedPos);
        rightExtendoServo.setPosition(extendoRetractedPos);
        rightArm.setPosition(rightArmInit);
        leftArm.setPosition(leftArmInit);
        clawWrist.setPosition(wristInitPos);
        clawServo.setPosition(clawOpenPos);

        intakeWrist.setPosition(intakeWrist_RetractedPos);
        intake.setPower(intakeSlowPower);
        intakeRight.setPosition(fourBarRetractedPos);
        intakeLeft.setPosition(fourBarRetractedPos);


        double targets = 0;

        // CURRENT STATES
        boolean going_down = false;
        boolean scissor_extended = false;
        boolean intake_reversed = false;
        boolean intake_running = false;
        boolean extended_to_basket = false;
        boolean clawReset = true;

        // BUTTON RELEASES
        boolean gamepad1_rightBumperReleased = true;
        boolean gamepad1_rightTriggerReleased = true;
        boolean gamepad1_leftBumperReleased = true;
        boolean gamepad1_dPadDownReleased = true;
        boolean gamepad1_dPadUpReleased = true;
        boolean gamepad1_leftTriggerReleased = true;
        boolean gamepad2_xReleased = true;
        boolean gamepad2_yReleased = true;
        boolean gamepad1_xReleased = true;
        boolean gamepad1_bReleased = true;
//TESTING
        boolean gamepad1_yReleased = true;
        boolean gamepad1_aReleased = true;
        boolean gamepad1_dPadLeftReleased = true;
        boolean gamepad1_dPadRightReleased = true;
        boolean gamepad1_leftJoystickButtonReleased = true;
        boolean gamepad1_rightJoystickButtonReleased = true;



        waitForStart();

        if (isStopRequested()) {
            intakeRight.resetDeviceConfigurationForOpMode();
            intakeLeft.resetDeviceConfigurationForOpMode();
            return;
        }

        while (opModeIsActive()) {
            double slidesleftpos = slidesLeft.getCurrentPosition();
            double powerLeft = controller.calculate(slidesleftpos, targets);

            double slidesrightpos = slidesRight.getCurrentPosition();
            double powerRight = controller.calculate(slidesrightpos, targets);

            slidesLeft.setPower(powerLeft);
            slidesRight.setPower(powerRight);


            telemetry.addData("Arm Pos", clawWrist.getPosition());
            telemetry.addData("Left Wrist Pos", leftArm.getPosition());
            telemetry.addData("Right Wrist Pos", rightArm.getPosition());
            telemetry.addData("Four Bar Pos", intakeLeft.getPosition());
            telemetry.addData("Intake Angle", intakeWrist.getPosition());
            telemetry.addData("Extendo Pos", leftExtendoServo.getPosition());
           // telemetry.addData("Slides Left power", powerLeft);
            // telemetry.addData("Slides right power", powerRight);
            telemetry.addData("Slides Left", slidesLeft.getCurrentPosition());
            telemetry.addData("Slides Right", slidesRight.getCurrentPosition());
            telemetry.update();

            // GAMEPAD 1 CONTROLS

            //ARM MOTOR POSITION TESTING
            if (gamepad1_aReleased && gamepad1.a) {
                double currPosition = leftArm.getPosition();
                leftArm.setPosition(currPosition - 0.05);
                rightArm.setPosition(currPosition - 0.05);
                gamepad1_aReleased = false;
            }
            if (gamepad1_yReleased && gamepad1.y) {
                double currPosition = leftArm.getPosition();
                leftArm.setPosition(currPosition + 0.05);
                rightArm.setPosition(currPosition + 0.05);
                gamepad1_yReleased = false;
            }
            if (gamepad1.x && gamepad1_xReleased) {
                double currPosition = clawWrist.getPosition();
                clawWrist.setPosition(currPosition - 0.05);
                gamepad1_xReleased = false;
            }
            if (gamepad1.b && gamepad1_bReleased) {
                double currPosition = clawWrist.getPosition();
                clawWrist.setPosition(currPosition + 0.05);
                gamepad1_bReleased = false;
            }

            
            if (gamepad1_dPadDownReleased && gamepad1.dpad_down) {
                double currPosition = intakeRight.getPosition();
                intakeLeft.setPosition(currPosition - 0.01);
                intakeRight.setPosition(currPosition - 0.01);
                gamepad1_dPadDownReleased = false;
            }
            if (gamepad1_dPadUpReleased && gamepad1.dpad_up) {
                double currPosition = intakeRight.getPosition();
                intakeLeft.setPosition(currPosition + 0.01);
                intakeRight.setPosition(currPosition + 0.01);
                gamepad1_dPadUpReleased = false;
            }
            if (gamepad1_dPadLeftReleased && gamepad1.dpad_left) {
                double currPosition = intakeWrist.getPosition();
                intakeWrist.setPosition(currPosition - 0.01);
                gamepad1_dPadLeftReleased = false;
            }
            if (gamepad1_dPadRightReleased && gamepad1.dpad_right) {
                double currPosition = intakeWrist.getPosition();
                intakeWrist.setPosition(currPosition + 0.01);
                gamepad1_dPadRightReleased = false;
            }


            if (gamepad1.right_bumper && !scissor_extended && gamepad1_rightBumperReleased) {
                intakeWrist.setPosition(intakeWrist_IntakingPos);
                rightExtendoServo.setPosition(extendoExtendedPos);
                leftExtendoServo.setPosition(extendoExtendedPos);
                intakeRight.setPosition(fourBarExtendedPos);
                intakeLeft.setPosition(fourBarExtendedPos);
                intake.setPower(intakePower);
                scissor_extended = true;
                intake_running = false;
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

            if (gamepad1.right_trigger != 0 && gamepad1_rightTriggerReleased) {
                clawServo.setPosition(clawClosePos);
                gamepad1_rightTriggerReleased = false;
            }
            if (gamepad1.left_trigger != 0 && gamepad1_leftTriggerReleased) {
                clawServo.setPosition(clawOpenPos);
                gamepad1_leftTriggerReleased = false;

            }
            if (gamepad1_leftJoystickButtonReleased && gamepad1.left_stick_button) {
                targets = midLevel;
                gamepad1_leftJoystickButtonReleased = false;
            }
            if (gamepad1_rightJoystickButtonReleased && gamepad1.right_stick_button) {
                targets = highLevel;
                gamepad1_rightJoystickButtonReleased = false;
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
            if (!gamepad1.dpad_right) {
                gamepad1_dPadRightReleased = true;
            }
            if (!gamepad1.dpad_left) {
                gamepad1_dPadLeftReleased = true;
            }
            if (gamepad1.left_trigger == 0) {
                gamepad1_leftTriggerReleased = true;
            }

            //ARM MOTOR POSITION TESTING
            if (!gamepad1.a) {
                gamepad1_aReleased = true;
            }
            if (!gamepad1.y) {
                gamepad1_yReleased = true;
            }
            if (!gamepad2.x) {
                gamepad2_xReleased = true;
            }
            if (!gamepad1.x) {
                gamepad1_xReleased = true;
            }
            if (!gamepad1.b) {
                gamepad1_bReleased = true;
            }
            if (!gamepad2.y) {
                gamepad2_yReleased = true;
            }
            if (gamepad1.right_trigger == 0) {
                gamepad1_rightTriggerReleased = true;
            }
            if (gamepad1.left_trigger == 0) {
                gamepad1_leftTriggerReleased = true;
            }
            if (!gamepad1.left_stick_button) {
                gamepad1_leftJoystickButtonReleased = true;
            }
            if (!gamepad1.right_stick_button) {
                gamepad1_rightJoystickButtonReleased = true;
            }



        }
    }
}

*/
