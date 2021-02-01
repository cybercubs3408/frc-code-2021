/*

    Notes:
    - DON'T USE THIS CLASS. IT'S VERY BAD AND NOT GOOD.

*/


package frc.robot;

// Imports WPI classes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

// Imports CTRE classes.
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

// Imports REV Robotics classes.
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;


public class OldRobot extends TimedRobot {

  // Joysticks
  Joystick leftJoystick = new Joystick(0);
  Joystick rightJoystick = new Joystick(1);
  Joystick thirdJoystick = new Joystick(2);

  // Intake and Hopper
  TalonSRX intakeMotorController = new TalonSRX(0);
  TalonSRX leftHopperMotorController = new TalonSRX(1);
  TalonSRX rightHopperMotorController = new TalonSRX(2);

  // Spinner
  TalonSRX spinnerMotorController = new TalonSRX(3);

  // Lift
  TalonSRX leftLiftMotorController = new TalonSRX(4);
  TalonSRX rightLiftMotorController = new TalonSRX(5);

  // Drive Train
  CANSparkMax frontLeftDriveTrainMotorController = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax backLeftDriveTrainMotorController = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax frontRightDriveTrainMotorController = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax backRightDriveTrainMotorController = new CANSparkMax(9, MotorType.kBrushless);
  CANEncoder frontLeftDriveTrainMotorEncoder = frontLeftDriveTrainMotorController.getEncoder();
  CANEncoder backLeftDriveTrainMotorEncoder = backLeftDriveTrainMotorController.getEncoder();
  CANEncoder frontRightDriveTrainMotorEncoder = frontRightDriveTrainMotorController.getEncoder();
  CANEncoder backRightDriveTrainMotorEncoder = backRightDriveTrainMotorController.getEncoder();

  // Shooter
  CANSparkMax leftShooterMotorController = new CANSparkMax(44, MotorType.kBrushless);
  CANSparkMax rightShooterMotorController = new CANSparkMax(55, MotorType.kBrushless);
  CANEncoder leftShooterMotorEncoder = leftShooterMotorController.getEncoder();
  CANEncoder rightShooterMotorEncoder = rightShooterMotorController.getEncoder();
  CANPIDController shooterPID = leftShooterMotorController.getPIDController();
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  // Limelight
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry horizontalOffsetEntry = limelightTable.getEntry("tx");
  NetworkTableEntry verticalOffsetEntry = limelightTable.getEntry("ty");
  NetworkTableEntry targetAreaEntry = limelightTable.getEntry("ta");
  NetworkTableEntry targetValidityEntry = limelightTable.getEntry("tv");
  double horizontalOffset;                // the Limelight's horizontal offset from the target, in degrees
  double verticalOffset;                  // the Limelight's vertical offset from the target, in degrees
  double targetArea;                      // the area of the target as seen by the Limelight, in square pixels
  double targetValidity;
  final double limelightHeight = 33.5;    // height of the Limelight from the ground, in inches
  final double targetHeight = 97;         // height of the target from the ground, in inches
  final double limelightAngle = 3;        // angle of the Limelight with respect to the horizontal, in degrees
  double targetDistance;                  // horizontal distance from the Limelight to the base of the target, in inches

  // Color Sensor
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  ColorMatch colorMatcher = new ColorMatch(); 
  Color detectedColor;
  ColorMatchResult match;
  String colorString;
  String wantedColorString;
  String gameDataString;
  final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // Compressor
  Compressor compressor = new Compressor();
  Solenoid intakeArmSolenoid = new Solenoid(2);
  DoubleSolenoid ratchetSolenoid = new DoubleSolenoid(0, 1);

  /**
   * Control the robot upon enabling the robot.
   */
  @Override
  public void robotInit() {

    // Resets the state of the compressor.
    compressor.clearAllPCMStickyFaults();

    // Starts the compressor.
    compressor.start();

    // Inverts and links motors if necessary.
    intakeMotorController.setInverted(true);
    leftHopperMotorController.setInverted(false);
    rightHopperMotorController.setInverted(true);
    spinnerMotorController.setInverted(false);
    leftLiftMotorController.setInverted(true);
    rightLiftMotorController.setInverted(false);
    frontLeftDriveTrainMotorController.setInverted(false);
    backLeftDriveTrainMotorController.setInverted(false);
    frontRightDriveTrainMotorController.setInverted(true);
    backRightDriveTrainMotorController.setInverted(true);
    rightShooterMotorController.follow(leftShooterMotorController, true);

    // Adds colors to the color matcher
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    // Define PID Coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 6000;
    
    // Set PID Coefficients
    shooterPID.setP(kP);  
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setIZone(kIz);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(kMinOutput, kMaxOutput);

    // Display PID Coefficients
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  /**
   * Control the robot upon enabling autonomous mode.
   */
  @Override
  public void autonomousInit() {

    // Resets drive-train encoders.
    frontLeftDriveTrainMotorEncoder.setPosition(0);
    backLeftDriveTrainMotorEncoder.setPosition(0);
    frontRightDriveTrainMotorEncoder.setPosition(0);
    backRightDriveTrainMotorEncoder.setPosition(0);

    // Drives away from the target.
    driveAutonomously(0.1, 35, false, true);

  }

  /**
   * Control the robot after enabling autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {

    // Checks if the robot has driven far enough away, aims at the target, and starts the shooter.
    if (frontLeftDriveTrainMotorEncoder.getPosition() < 35) {
      driveAutonomously(0);
      prepareToShoot(false, true);
      driveAutonomously(0);
    }

    // Checks if the robot is aiming at the target to shoot.
    if (Math.abs(horizontalOffset) < 0.5) {
      moveHopperUp(0.3);
    }

  }

  /**
   * Control the robot upon enabling tele-operated mode.
   */
  @Override
  public void teleopInit() {

    // Stops the robot.
    driveAutonomously(0);

    // Starts the camera streams.
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);

    // Resets drive-train encoders.
    frontLeftDriveTrainMotorEncoder.setPosition(0);
    backLeftDriveTrainMotorEncoder.setPosition(0);
    frontRightDriveTrainMotorEncoder.setPosition(0);
    backRightDriveTrainMotorEncoder.setPosition(0);

  }

  /**
   * Control the robot after enabling tele-operated mode.
   */
  @Override
  public void teleopPeriodic() {

    // Allows the robot to be driven with tank-drive controls throughout teleop mode.
    driveRobot();

    if (leftJoystick.getRawButton(1)) {
      prepareToShoot(false, true);
    }

    // What's the point of this again?
    if (leftJoystick.getRawButtonPressed(1)) {
      prepareToShoot(false, true);
    }

    if (rightJoystick.getRawButton(1)) {
      moveHopperUp(0.32);
    }

    if (rightJoystick.getRawButton(2)) {
      freeSpin(0.3);
    }

    if (rightJoystick.getRawButton(3)) {
      beginRotationControl(true, false);
    }

    if (rightJoystick.getRawButton(4)) {
      beginPositionControl(true, false);
    }

    if (thirdJoystick.getRawAxis(1) > 0.9) {
      raiseIntakeArm();
    }

    if (thirdJoystick.getRawAxis(1) < -0.9) {
      dropIntakeArm();
    }

    if (thirdJoystick.getRawAxis(2) > 0.9) {
      outtake(0.5);
    }

    if (thirdJoystick.getRawAxis(3) > 0.9) {
      intake(0.5);
    }

    if (thirdJoystick.getRawButton(1)) {
      moveHopperUp(0.4);
    }

    if (thirdJoystick.getRawButton(5)) {
      moveHopperDown(0.4);
    }

    if (thirdJoystick.getRawButton(4)) {
      moveHopperUp(0.2);
    }

    if (thirdJoystick.getRawButton(6)) {
      moveHopperUp(0.2);
    }

    if (thirdJoystick.getPOV() == 270) {
      freeRatchet();
    }

    if (thirdJoystick.getPOV() == 90) {
      lockRatchet();
    }

    if (thirdJoystick.getPOV() == 0) {
      moveLiftUp(0.2);
    }

    if (thirdJoystick.getPOV() == 180) {
      moveLiftDown(0.2);
    }

    // Stops the shooter
    if (leftJoystick.getRawButton(2)) {
      leftShooterMotorController.set(0);
    }

    // Stops the intake, hopper, spinner, lift, and shooter motors.
    stopMechanisms();

  }

  /**
   * Control the robot after enabling test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Control the robot upon disabling the robot.
   */
  @Override
  public void disabledInit() {
    driveAutonomously(0);
    stopMechanisms();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // Helper Methods //

  /**
   * Raise the intake arm.
   */
  public void raiseIntakeArm() {
    intakeArmSolenoid.set(true);
  }

  /**
   * Drop the intake arm.
   */
  public void dropIntakeArm() {
    intakeArmSolenoid.set(false);
  }

  /**
   * Spin the intake inward.
   * @param power The unsigned power to apply to the motor controller.
   */
  public void intake(double power) {
    intakeMotorController.set(ControlMode.PercentOutput, Math.abs(power));
  }

  /**
   * Spin the intake outward.
   * @param power The unsigned power to apply to the motor controller.
   */
  public void outtake(double power) {
    intakeMotorController.set(ControlMode.PercentOutput, -Math.abs(power));
  }

  /**
   * Move the hopper inward.
   * @param power The unsigned power to apply to the motor controllers.
   */
  public void moveHopperUp(double power) {
    leftHopperMotorController.set(ControlMode.PercentOutput, Math.abs(power));
    rightHopperMotorController.set(ControlMode.PercentOutput, Math.abs(power));
  }

  /**
   * Move the hopper outward.
   * @param power The unsigned power to apply to the motor controllers.
   */
  public void moveHopperDown(double power) {
    leftHopperMotorController.set(ControlMode.PercentOutput, -Math.abs(power));
    rightHopperMotorController.set(ControlMode.PercentOutput, -Math.abs(power));
  }

  /**
   * Freely spin the spinner motor.
   * @param power The signed power to apply to the motor controller.
   */
  public void freeSpin(double power) {
    spinnerMotorController.set(ControlMode.PercentOutput, power);
  }

  /**
   * Update color variables and display them on Driver Station and SmartDashboard if necessary.
   * @param driverStationDisplay A boolean representing whether or not to display the current color on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display values on SmartDashboard.
   */
  public void updateColorVariables(boolean driverStationDisplay, boolean smartDashboardDisplay) {

    // Defines what color the color sensor is looking at.
    detectedColor = colorSensor.getColor();
    match = colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = new String("B");
    } else if (match.color == kRedTarget) {
      colorString = new String("R");
    } else if (match.color == kGreenTarget) {
      colorString = new String("G");
    } else if (match.color == kYellowTarget) {
      colorString = new String("Y");
    } else {
      colorString = new String("U");
    }

    // Defines what color the game's color sensor should see during POSITION CONTROL.
    gameDataString = DriverStation.getInstance().getGameSpecificMessage();
    if (gameDataString.equals("B")) {
      wantedColorString = new String("R");
    } else if (gameDataString.equals("R")) {
      wantedColorString = new String("B");
    } else if (gameDataString.equals("G")) {
      wantedColorString = new String("Y");
    } else if (gameDataString.equals("Y")) {
      wantedColorString = new String("G");
    } else {
      wantedColorString = new String("");
    }

    // Reports the detected color to Driver Station only if driverStationDisplay is true.
    if (driverStationDisplay) {
      DriverStation.reportWarning(colorString, false);
    }

    // Reports the detected color, the wanted color, and the game data to SmartDashboard if smartDashboardDisplay is true.
      // The detected color is the color the robot's color sensor is currently looking at.
      // The game data is the color the game's color sensor should see during POSITION CONTROL.
      // The wanted color is the color the robot's color sensor should see so that the game data color aligns with the game's color sensor.
    if (smartDashboardDisplay) {
      SmartDashboard.putString("Detected Color", colorString);
      SmartDashboard.putString("Wanted Color", wantedColorString);
      SmartDashboard.putString("Game Data", gameDataString);
    }

  }

  /**
   * Spin the CONTROL PANEL for more than 3 revolutions but fewer than 5 revolutions.
   * @param driverStationDisplay A boolean representing whether or not to display the current color on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display values on SmartDashboard.
   */
  public void beginRotationControl(boolean driverStationDisplay, boolean smartDashboardDisplay) {

    /*
    ROTATION CONTROL involves spinning the CONTROL PANEL for more than 3 revolutions but fewer than 5 revolutions.
    This code works by first spinning the wheel to red.
    Then, it spin until it sees blue, then red again, then blue again, etc., until the wheel has been spun around 4 times.
    The outer loop loops 16 times, because each revolution requires 4 "hits." (A "hit" is when the robot's color sensor sees either red or blue.) 
    */

    // Loops through 16 iterations in order to spin the CONTROL PANEL approximately 4 times.
    for (int i = 0; i < 16; i++) {

      updateColorVariables(driverStationDisplay, smartDashboardDisplay);

      if (leftJoystick.getRawButton(2)) {
        freeSpin(0);
        break;
      }

      // Checks if the detected color is red in order to spin the CONTROL PANEL to blue.
      if (match.color == kRedTarget) {

        // Loops as long as the detected color is not blue in order to spin the CONTROL PANEL to blue.
        while (match.color != kBlueTarget) {

          updateColorVariables(driverStationDisplay, smartDashboardDisplay);
          freeSpin(0.6);

          if (leftJoystick.getRawButton(2)) {
            freeSpin(0);
            break;
          }

        }

      }

      // Checks if the detected color is blue in order to spin the CONTROL PANEL to red.
      else if (match.color == kBlueTarget) {

        // Loops as long as the detected color is not red in order to spin the CONTROL PANEL to red.
        while (match.color != kRedTarget) {

          updateColorVariables(driverStationDisplay, smartDashboardDisplay);
          freeSpin(0.6);

          if (leftJoystick.getRawButton(2)) {
            freeSpin(0);
            break;
          }

        }

      }

      // Checks if the detected color is neither blue nor red in order to spin the CONTROL PANEL to red.
      else {

        // Loops as long as the detected color is neither red nor blue in order to spin the CONTROL PANEL to red.
        while (match.color != kRedTarget) {

          updateColorVariables(driverStationDisplay, smartDashboardDisplay);
          freeSpin(0.6);

          if (leftJoystick.getRawButton(2)) {
            freeSpin(0);
            break;
          }

        }

      }

    }

    freeSpin(0);

  }

  /**
   * Spin the CONTROL PANEL until the specified color is aligned with the ARENA's color sensor.
   * @param driverStationDisplay A boolean representing whether or not to display the current color on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display color values on SmartDashboard.
   */
  public void beginPositionControl(boolean driverStationDisplay, boolean smartDashboardDisplay) {

    /*
    POSITION CONTROL involves spinning the CONTROL PANEL so that the game's color sensor aligns with a "specified color" (given through game data).
    Since the robot is positioned differently from the game's color sensor, we define a "wanted color," such that if the robot's color sensor sees the "wanted color,"
      the game's color sesor sees the "specified color." For example, if the game wants to see red, the robot should see blue.
    The updateColorVariables method takes care of defining the "specified color" and the "wanted color."
    This code works by spinning the spinner mechanism until the color sensor sees the "wanted" color.
    */

    // Updates the color variables.
    updateColorVariables(driverStationDisplay, smartDashboardDisplay);

    // Loops as long as the detected color is not the wanted color in order to align the specified color with the ARENA's color sensor.
    while (!colorString.equals(wantedColorString)) {

      updateColorVariables(driverStationDisplay, smartDashboardDisplay);
      freeSpin(0.6);

      if (leftJoystick.getRawButton(2)) {
        freeSpin(0);
        break;
      }

    }

    freeSpin(0);

  }

  /**
   * Free the ratchet.
   */
  public void freeRatchet() {
    ratchetSolenoid.set(Value.kReverse);
    thirdJoystick.setRumble(RumbleType.kLeftRumble, 0);
    thirdJoystick.setRumble(RumbleType.kRightRumble, 0);
  }  

  /**
   * Lock the ratchet.
   */
  public void lockRatchet() {
    ratchetSolenoid.set(Value.kForward);
    thirdJoystick.setRumble(RumbleType.kLeftRumble, 1);
    thirdJoystick.setRumble(RumbleType.kRightRumble, 1);
  }

  /**
   * Move the lift up.
   * @param power The unsigned power to apply to the motor controllers.
   */
  public void moveLiftUp(double power) {
    leftLiftMotorController.set(ControlMode.PercentOutput, Math.abs(power));
    rightLiftMotorController.set(ControlMode.PercentOutput, Math.abs(power));
  }

  /**
   * Move the lift down.
   * @param power The unsigned power to apply to the motor controllers.
   */
  public void moveLiftDown(double power) {
    leftLiftMotorController.set(ControlMode.PercentOutput, -Math.abs(power));
    rightLiftMotorController.set(ControlMode.PercentOutput, -Math.abs(power));
  }

  /**
   * Drive the robot using tank-drive controls with a cubic power curve.
   */
  public void driveRobot() {
    frontLeftDriveTrainMotorController.set(Math.pow(rightJoystick.getRawAxis(1), 3));
    backLeftDriveTrainMotorController.set(Math.pow(rightJoystick.getRawAxis(1), 3));
    frontRightDriveTrainMotorController.set(Math.pow(leftJoystick.getRawAxis(1), 3));
    backRightDriveTrainMotorController.set(Math.pow(leftJoystick.getRawAxis(1), 3));
  }

  /**
   * Drive the robot autonomously in a straight line.
   * @param power The signed power to apply to the motor controllers
   */
  public void driveAutonomously(double power) {
    frontLeftDriveTrainMotorController.set(power);
    backLeftDriveTrainMotorController.set(power);
    frontRightDriveTrainMotorController.set(power);
    backRightDriveTrainMotorController.set(power);
  }

  /**
   * Drive the robot autonomously in a straight line.
   * @param power The signed power to apply to the motor controllers.
   * @param distance The distance, in <units>, to drive.
   * @param driverStationDisplay A boolean representing whether or not to display the current drive encoder position on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display the current drive encoder position on SmartDashboard.
   */
  public void driveAutonomously(double power, double distance, boolean driverStationDisplay, boolean smartDashboardDisplay) {

    // Resets the encoder.
    frontLeftDriveTrainMotorEncoder.setPosition(0);

    // Drives until the encoder's position reaches the specified distance.
    while (Math.abs(frontLeftDriveTrainMotorEncoder.getPosition()) < Math.abs(distance)) {
      frontLeftDriveTrainMotorController.set(power);
      backLeftDriveTrainMotorController.set(power);
      frontRightDriveTrainMotorController.set(power);
      backRightDriveTrainMotorController.set(power);

      // Displays the encoder position on Driver Station only if driverStationDisplay is true.
      if (driverStationDisplay) {
        DriverStation.reportWarning(Double.toString(frontLeftDriveTrainMotorEncoder.getPosition()), false);
      }

      // Displays the encoder position on SmartDashboard only if smartDashboardDisplay is true.
      if (smartDashboardDisplay) {
        SmartDashboard.putNumber("Drive Encoder", frontLeftDriveTrainMotorEncoder.getPosition());
      }

    }
  }

  /**
   * Turn the robot autonomously.
   * @param direction The direction to turn the robot ("LEFT" or "RIGHT").
   * @param power The unsigned power to apply to the motor controllers.
   */
  public void turnAutonomously(String direction, double power) {

    if (direction.equals("LEFT")) {
      frontLeftDriveTrainMotorController.set(power);
      backLeftDriveTrainMotorController.set(power);
      frontRightDriveTrainMotorController.set(-power);
      backRightDriveTrainMotorController.set(-power);
    } else if (direction.equals("RIGHT")) {
      frontLeftDriveTrainMotorController.set(-power);
      backLeftDriveTrainMotorController.set(-power);
      frontRightDriveTrainMotorController.set(power);
      backRightDriveTrainMotorController.set(power);
    }

  }

  /**
   * Turn the robot autonomously.
   * @param direction The direction to turn the robot ("LEFT" or "RIGHT").
   * @param power The unsigned power to apply to the motor controllers.
   * @param angle The signed angle, in <units>, to turn.
   * @param driverStationDisplay A boolean representing whether or not to display the current gyroscope angle on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display the current gyroscope angle on SmartDashboard.
   */
  public void turnAutonomously(String direction, double power, double angle, boolean driverStationDisplay, boolean smartDashboardDisplay) {

    // This code doesn't exist yet because I don't remember if the robot has a gyro.

  }
  
  /**
   * Update Limelight variables and display them on Driver Station and SmartDashboard if necessary.
   * @param driverStationDisplay A boolean representing whether or not to display the target validity on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display Limelight values on SmartDashboard.
   */
  public void updateLimelightVariables(boolean driverStationDisplay, boolean smartDashboardDisplay) {

    horizontalOffset = horizontalOffsetEntry.getDouble(0.0);
    verticalOffset = verticalOffsetEntry.getDouble(0.0);
    targetArea = targetAreaEntry.getDouble(0.0);
    targetValidity = targetValidityEntry.getDouble(0.0);    
    targetDistance = (targetHeight - limelightHeight) / Math.tan((limelightAngle + verticalOffset) * Math.PI / 180);

    // Reports whether or not a target is found to Driver Station only if driverStationDisplay is true.
    if (driverStationDisplay) {
      DriverStation.reportWarning(Double.toString(targetValidity), false);
    }

    // Reports various Limelight variables to SmartDashboard only if smartDashboardDisplay is true.
    if (smartDashboardDisplay) {
      SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
      SmartDashboard.putNumber("Vertical Offset", verticalOffset);
      SmartDashboard.putNumber("Target Area", targetArea);
      SmartDashboard.putNumber("Target Validity", targetValidity);
      SmartDashboard.putNumber("Target Distance", targetDistance);
    }

  }

  /**
   * Convert the distance from the target to an autonomous shooter power.
   * @param targetDistance The distance from the target, in inches.
   * @return The power to be applied to the autonomous shooter.
   */
  public double convertTargetDistanceToAutonomousShooterPower(double targetDistance) {

    updateLimelightVariables(false, false);
    double output = 1.5 / ((targetDistance / 12) - 10) + 0.016 * (targetDistance / 12) + 0.20;   // Improve this function somehow.
    return output; 

  }

  /**
   * Align the robot with the target and spin the shooter motors.
   * @param driverStationDisplay A boolean representing whether or not to display the target validity on Driver Station.
   * @param smartDashboardDisplay A boolean representing whether or not to display Limelight values on SmartDashboard.
   */
  public void prepareToShoot(boolean driverStationDisplay, boolean smartDashboardDisplay) {

    updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);

    // New PID Code:

    // Read PID Coefficients
    /*
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // Reset PID Coefficients if they have changed.
    if (p != kP) {
      shooterPID.setP(p);
      kP = p;
    }
    if (i != kI) {
      shooterPID.setI(i);
      kI = i;
    }
    if (d != kD) {
      shooterPID.setD(d);
      kD = d;
    }
    if (iz != kIz) {
      shooterPID.setIZone(iz);
      kIz = iz;
    }
    if (ff != kFF) {
      shooterPID.setFF(ff); 
      kFF = ff;
    }
    if (min != kMinOutput || max != kMaxOutput) {
      shooterPID.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    double setPoint = convertTargetDistanceToAutonomousShooterPower(targetDistance) * maxRPM;
    shooterPID.setReference(setPoint, ControlType.kVelocity);
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", leftShooterMotorEncoder.getVelocity());
    */

    if (targetValidity == 1) {
      leftShooterMotorController.set(convertTargetDistanceToAutonomousShooterPower(targetDistance));
    } else {
      leftShooterMotorController.set(1);
    }

    // Reports the shooter power to SmartDashboard only if smartDashboardDisplay is true.
    if (smartDashboardDisplay) {
      SmartDashboard.putNumber("Shooter Power", convertTargetDistanceToAutonomousShooterPower(targetDistance));
    }

    if (leftJoystick.getRawButton(2)) {
      leftShooterMotorController.set(0);
    }

    // Aligns the robot with the target and turns on the shooter motors.
    while (Math.abs(horizontalOffset) > 0.1) {

      updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);
      leftShooterMotorController.set(convertTargetDistanceToAutonomousShooterPower(targetDistance));

      // Checks if the horizontal offset is greater than 0, meaning the robot is to the right of the target, to align the robot by turning it left.
      if (horizontalOffset > 0) {
        turnAutonomously("LEFT", 0.09);
      }
      
      // Checks if the horizontal offset is less than 0, meaning the robot is to the left of the target, to align the robot by turning it right.
      if (horizontalOffset < 0) {
        turnAutonomously("RIGHT", 0.09);
      }

      if (leftJoystick.getRawButton(2)) {
        driveAutonomously(0);
        break;
      }

    }

    driveAutonomously(0);

  }

  /**
   * Stop the intake, hopper, spinner, lift, and shooter motors.
   */
  public void stopMechanisms() {

    intakeMotorController.set(ControlMode.PercentOutput, 0);
    leftHopperMotorController.set(ControlMode.PercentOutput, 0);
    rightHopperMotorController.set(ControlMode.PercentOutput, 0);
    spinnerMotorController.set(ControlMode.PercentOutput, 0);
    leftLiftMotorController.set(ControlMode.PercentOutput, 0);
    rightLiftMotorController.set(ControlMode.PercentOutput, 0);
    leftShooterMotorController.set(0);
    rightShooterMotorController.set(0);

  }

}