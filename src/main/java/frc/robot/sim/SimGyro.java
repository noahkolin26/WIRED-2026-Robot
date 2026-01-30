package frc.robot.sim;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Simple simulated gyro for WPILib 2026+.
 * Store angle in degrees (CCW+), and expose Rotation2d for field-oriented drive.
 */
public class SimGyro {
  private double angleDeg = 0.0;
  private double rateDegPerSec = 0.0;

  /** Sets the absolute heading in degrees (CCW+). */
  public void setAngleDegrees(double angleDegrees) {
    this.angleDeg = angleDegrees;
  }

  /** Optional: set angular rate in degrees/sec (CCW+). */
  public void setRateDegreesPerSec(double rateDegreesPerSec) {
    this.rateDegPerSec = rateDegreesPerSec;
  }

  /** Adds a delta to the current angle (degrees). */
  public void addAngleDegrees(double deltaDegrees) {
    this.angleDeg += deltaDegrees;
  }

  /** Returns heading in degrees (CCW+). */
  public double getAngle() {
    return angleDeg;
  }

  /** Returns angular rate in degrees/sec (CCW+). */
  public double getRate() {
    return rateDegPerSec;
  }

  /** Returns heading as a Rotation2d (preferred for WPILib field-oriented APIs). */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(angleDeg);
  }

  /** Zeros the gyro. */
  public void reset() {
    angleDeg = 0.0;
    rateDegPerSec = 0.0;
  }
}
