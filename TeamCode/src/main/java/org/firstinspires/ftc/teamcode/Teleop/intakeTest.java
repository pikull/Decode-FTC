package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class intakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        waitForStart();
        while (opModeIsActive()) {
        if(gamepad1.a){
            if(intake.getPower() <= 1 && intake.getPower() > 0){
                intake.setPower(0);
            }
            else if(intake.getPower() == 0){
                intake.setPower(1);
            }
        }

    }
    }
}
