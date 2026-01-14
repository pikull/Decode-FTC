package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Motor")
public class motors extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
       DcMotorEx flywheelMotorL = hardwareMap.get(DcMotorEx.class, "leftShooter");
       DcMotorEx flywheelMotorR = hardwareMap.get(DcMotorEx.class, "rightShooter");
       flywheelMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
       flywheelMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       flywheelMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       Servo outake = hardwareMap.servo.get("outakeS");
        CRServo intakeS = hardwareMap.get(CRServo.class,"intakeS");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.a){
                flywheelMotorL.setVelocity(flywheelMotorL.getVelocity()+20);
                flywheelMotorR.setVelocity(flywheelMotorR.getVelocity()+20);
            }
            if(gamepad1.b){
                flywheelMotorL.setVelocity(flywheelMotorL.getVelocity()-20);
                flywheelMotorR.setVelocity(flywheelMotorR.getVelocity()-20);
            }
            if(gamepad1.right_bumper){
                outake.setPosition(outake.getPosition()+0.01);
            }
            if(gamepad1.left_bumper){
                outake.setPosition(outake.getPosition()-0.01);
            }
            intake.setPower(-1);
            intakeS.setPower(1);


            telemetry.addData("flywheelMotorR",flywheelMotorR.getVelocity());
            telemetry.addData(" flywheelMotorL", flywheelMotorL.getVelocity());
            telemetry.addData("servo",outake.getPosition());
            telemetry.update();

        }
    }

}
