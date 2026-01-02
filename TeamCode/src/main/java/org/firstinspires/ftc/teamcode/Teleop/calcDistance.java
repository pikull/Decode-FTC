package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.controller.PIDFController;


import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "calcDistance")
public class calcDistance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


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
                telemetry.addData("ta", result.getTa());
                telemetry.update();
            }
        }
    }
}