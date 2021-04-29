// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  // The motors on the left side of the drive.
  private final SpeedControllerGroup leftMotors =
      new SpeedControllerGroup(
          new PWMSparkMax(DriveConstants.frontLeftMotorID),
          new PWMSparkMax(DriveConstants.backLeftMotorID));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup rightMotors =
      new SpeedControllerGroup(
          new PWMSparkMax(DriveConstants.frontRightMotorID),
          new PWMSparkMax(DriveConstants.backRightMotorID));

  // The robot's drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  // The left-side drive encoder
  private final Encoder leftEncoder =
      new Encoder(
          DriveConstants.leftEncoderIDs[0],
          DriveConstants.leftEncoderIDs[1],
          DriveConstants.leftEncoderReversed);

  // The right-side drive encoder
  private final Encoder rightEncoder =
      new Encoder(
          DriveConstants.rightEncoderIDs[0],
          DriveConstants.rightEncoderIDs[1],
          DriveConstants.rightEncoderReversed);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(DriveConstants.encoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.encoderDistancePerPulse);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed, true);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

}
