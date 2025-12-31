package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorTest")

public class MotorTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor luanch1 = hardwareMap.dcMotor.get("luanch1");
        DcMotor launch2 = hardwareMap.dcMotor.get("launch2");
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo lAngle = hardwareMap.servo.get("LaunchAngle");
        lAngle.setDirection(Servo.Direction.REVERSE);
        lAngle.setPosition(-1);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.x){
                luanch1.setPower(1);
                launch2.setPower(1);
            }

            while(gamepad1.a){
                lAngle.setPosition(lAngle.getPosition()+0.001);
            }
            while(gamepad1.b){
                lAngle.setPosition(lAngle.getPosition()- 0.001);
            }

        }

    }

}
