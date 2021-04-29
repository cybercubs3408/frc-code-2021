// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s).
 */
public class TankDrive extends CommandBase {
  private final DriveSubsystem drive;
  private final DoubleSupplier leftSpeed;
  private final DoubleSupplier rightSpeed;

  /**
   * Creates a new TankDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param left The control input for the left motors.
   * @param right The control input for the right motors.
   */
  public TankDrive(DriveSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
    drive = subsystem;
    leftSpeed = left;
    rightSpeed = right;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.tankDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  }

}
