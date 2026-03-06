package frc.robot.commands.Driving;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class VisionHeadingAssist extends Command {

    private final DriveSubsystem drive;
    private final LimelightVision vision;
    private final int targetTag;

    private final double kP = 0.02;       // proportional gain
    private final double maxRotation = 1.5; // rad/s
    private final double minRotation = 0.05; // rad/s
    private final LinearFilter txFilter = LinearFilter.movingAverage(5);

    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;

    /**
     * @param drive DriveSubsystem
     * @param vision LimelightVision
     * @param tagID the target AprilTag
     * @param forward DoubleSupplier for forward/backward input from driver
     * @param strafe DoubleSupplier for left/right input from driver
     */
    public VisionHeadingAssist(DriveSubsystem drive, LimelightVision vision, int tagID,
                               DoubleSupplier forward, DoubleSupplier strafe) {
        this.drive = drive;
        this.vision = vision;
        this.targetTag = tagID;
        this.forward = forward;
        this.strafe = strafe;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double tx = 0.0;

        // Only apply vision correction if we see the tag
        if (vision.seesTag(targetTag)) {
            tx = txFilter.calculate(vision.getTX());
        }

        // Simple proportional rotation controller
        double rotation = -tx * kP;

        // Apply minimum rotation if needed to overcome friction
        if (Math.abs(rotation) < minRotation && Math.abs(tx) > 1.0) {
            rotation = Math.copySign(minRotation, rotation);
        }

        // Clamp max rotation
        rotation = Math.max(-maxRotation, Math.min(maxRotation, rotation));

        // Drive with driver inputs for translation + vision rotation
        drive.drive(new ChassisSpeeds(
            forward.getAsDouble(),
            strafe.getAsDouble(),
            rotation
        ));
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
    }
}