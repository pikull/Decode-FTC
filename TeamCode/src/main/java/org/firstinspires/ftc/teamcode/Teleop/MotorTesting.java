/*package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="Motor Testing")
public class MotorTesting extends LinearOpMode {
    ;
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;





    public void runOpMode() {

//slidesleft.setDirection(DcMotor.Direction.REVERSE);



        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            while (gamepad2.a) {
                rightFront.setPower(1);
            }
            while (gamepad2.b) {
                rightBack.setPower(1);
            }
            while (gamepad2.x) {
                leftBack.setPower(1);
            }
            while (gamepad2.y) {
                leftFront.setPower(1);
            }

            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);


        }
    }
}

 */