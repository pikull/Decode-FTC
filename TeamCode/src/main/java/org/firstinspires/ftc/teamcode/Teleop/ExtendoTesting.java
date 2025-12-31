package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.PIDFController;


public class ExtendoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS

        Servo rightExtendoServo = hardwareMap.servo.get("rightServo");
        Servo leftExtendoServo = hardwareMap.servo.get("leftServo");
        Servo intakeAngle = hardwareMap.servo.get("intakeAngle");
        Servo fourBarRight = hardwareMap.servo.get("fourBarRight");
        Servo fourBarleft = hardwareMap.servo.get("fourBarLeft");


        //POWERS & POSITIONS //////////////////////////////////

        double extendoRetractedPos = 0.4;
        double extendoExtendedPos = 0.7;
        double intakeAngle_IntakingPos = 0.6;
        double intakeAngle_RetractedPos = 0;
        double fourBarRetractedPos = 0.25;
        double fourBarExtendedPos = 0.85;

        //INIATE MOTOR POSITIONS
        fourBarRight.setDirection(Servo.Direction.REVERSE);
        leftExtendoServo.setPosition(extendoRetractedPos);
        rightExtendoServo.setPosition(extendoRetractedPos);
        fourBarRight.setPosition(fourBarRetractedPos);
        fourBarleft.setPosition(fourBarRetractedPos);


        intakeAngle.setPosition(intakeAngle_RetractedPos);



//TESTING
        boolean gamepad1_yReleased = true;
        boolean gamepad1_aReleased = true;
        boolean gamepad1_rightTriggerReleased = true;
        boolean gamepad1_leftTriggerReleased = true;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        // GAMEPAD 1 CONTROLS

             //ARM MOTOR POSITION TESTING
            if (gamepad1_aReleased && gamepad1.a) {
                double currPosition = leftExtendoServo.getPosition();
                leftExtendoServo.setPosition(currPosition - 0.025);
                rightExtendoServo.setPosition(currPosition - 0.025);
                gamepad1_aReleased = false;
            }
            if (gamepad1_yReleased && gamepad1.y) {
                double currPosition = leftExtendoServo.getPosition();
                leftExtendoServo.setPosition(currPosition + 0.025);
                rightExtendoServo.setPosition(currPosition + 0.025);
                gamepad1_yReleased = false;
            }
            if (gamepad1.right_trigger != 0 && gamepad1_rightTriggerReleased){
                leftExtendoServo.setPosition(extendoExtendedPos);
                rightExtendoServo.setPosition(extendoExtendedPos);
                gamepad1_rightTriggerReleased = false;
            }
            if (gamepad1.left_trigger != 0 && gamepad1_leftTriggerReleased){
                leftExtendoServo.setPosition(extendoRetractedPos);
                rightExtendoServo.setPosition(extendoRetractedPos);
                gamepad1_leftTriggerReleased = false;
            }


            //MECANUM DRIVE

        // SCISSOR LIFT + INTAKE ///////////////

            // extends scissor lift, extends 4bar

            //ARM MOTOR POSITION TESTING
            if(!gamepad1.a){
                gamepad1_aReleased = true;
            }
            if(!gamepad1.y){
                gamepad1_yReleased = true;
            }
            if (gamepad1.right_trigger == 0) {
                gamepad1_rightTriggerReleased = true;
            }
            if (gamepad1.left_trigger == 0) {
                gamepad1_leftTriggerReleased = true;
            }






        }

    }
}