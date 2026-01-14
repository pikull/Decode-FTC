package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

// This class acts as a bridge between the Pinpoint hardware and Road Runner
public class PinpointLocalizer implements Localizer {
    public final GoBildaPinpointDriver odo;
    private Pose2d lastPose;

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // Ensure "pinpoint" matches your Driver Station configuration
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // 1. SET YOUR OFFSETS (in millimeters)
        // X offset: distance from center to the perpendicular pod
        // Y offset: distance from center to the parallel pod
        odo.setOffsets(0.0, 0.0);

        // 2. SET ENCODER RESOLUTION
        // Use goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // 3. SET DIRECTIONS (X encoder: FORWARD, Y encoder: FORWARD)
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        lastPose = initialPose;
    }

    @Override
    public Twist2dDual<Time> update() {
        odo.update();

        // Get current pose from Pinpoint (convert mm to inches)
        Pose2d currentPose = new Pose2d(
                new Vector2d(odo.getPosX() / 25.4, odo.getPosY() / 25.4),
                Rotation2d.exp(odo.getHeading())
        );

        // Calculate the twist (change in pose) - use proper vector subtraction
        Vector2d deltaPosition = currentPose.position.minus(lastPose.position);
        double deltaHeading = currentPose.heading.minus(lastPose.heading);
        
        // Get velocity from Pinpoint (convert mm/s to inches/s)
        Vector2d velocity = new Vector2d(odo.getVelX() / 25.4, odo.getVelY() / 25.4);
        double angularVelocity = odo.getHeadingVelocity();

        // Create the twist with position and velocity information
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[]{deltaPosition.x, velocity.x}),
                        new DualNum<Time>(new double[]{deltaPosition.y, velocity.y})
                ),
                new DualNum<Time>(new double[]{deltaHeading, angularVelocity})
        );

        lastPose = currentPose;
        return twist;
    }
}