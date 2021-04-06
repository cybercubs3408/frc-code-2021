/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
// import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.CharacterizationConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static Joystick controller;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controller = new Joystick(0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {

    Pose2d strikeIt = new Pose2d();

    driveSubsystem.resetEncoders();
    driveSubsystem.zeroHeading();
    driveSubsystem.resetOdometry(strikeIt);

    // Trajectory is read from Pathweaver .json file; place file in src/main/deploy
    Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/Straight2.wpilib.json")); 

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, driveSubsystem::getPose,
        new RamseteController(CharacterizationConstants.kRamseteB, CharacterizationConstants.kRamseteZeta),
        new SimpleMotorFeedforward(CharacterizationConstants.ksVolts, CharacterizationConstants.kvVoltSecondsPerMeter,
            CharacterizationConstants.kaVoltSecondsSquaredPerMeter),
        CharacterizationConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(CharacterizationConstants.kPDriveVel, 0, 0), 
        new PIDController(CharacterizationConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts, 
        driveSubsystem
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}
