/*

    Notes:
    - The second turnAutonomous method is empty because of a lack of gyroscope code.
    - Most of the autonomous code is not useful right now, I have left it in for now, 
      but it will be overwritten with new code for Pathweaver.
    - To Do: Add PID Control to the prepareToShoot method.
    - To Do: Investigate and solve issue of prepareToShoot method not activating.

*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends Mechanism {

    CANSparkMax frontLeft, backLeft, frontRight, backRight;
    CANEncoder frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder;

    /**
     * Constructs a Drivetrain object. Initializes the encoders on each motor and the recording/playback variables.
     * @param frontLeftID The ID of the front-left motor controller controlling the drivetrain.
     * @param backLeftID The ID of the back-left motor controller controlling the drivetrain.
     * @param frontRightID The ID of the front-right motor controller controlling the drivetrain.
     * @param backRightID The ID of the back-right motor controller controlling the drivetrain.
     */
    public Drivetrain(int frontLeftID, int backLeftID, int frontRightID, int backRightID) {

        frontLeft = new CANSparkMax(frontLeftID, MotorType.kBrushless);
        backLeft = new CANSparkMax(backLeftID, MotorType.kBrushless);
        frontRight = new CANSparkMax(frontRightID, MotorType.kBrushless);
        backRight = new CANSparkMax(backRightID, MotorType.kBrushless);
        frontLeftEncoder = frontLeft.getEncoder();
        backLeftEncoder = backLeft.getEncoder();
        frontRightEncoder = frontRight.getEncoder();
        backRightEncoder = backRight.getEncoder();

    }

    /**
     * Sets the inversion status of each motor controller.
     * @param frontLeftInversion A boolean representing whether or not to invert the front-left motor controller.
     * @param backLeftInversion A boolean representing whether or not to invert the back-left motor controller.
     * @param frontRightInversion A boolean representing whether or not to invert front-right motor controller. 
     * @param backRightInversion A boolean representing whether or not to invert the back-right motor controller.
     */
    public void setInversionStatus(boolean frontLeftInversion, boolean backLeftInversion, boolean frontRightInversion, boolean backRightInversion) {

        frontLeft.setInverted(frontLeftInversion);
        backLeft.setInverted(backLeftInversion);
        frontRight.setInverted(frontRightInversion);
        backRight.setInverted(backRightInversion);

    }

    /**
     * Resets the position of each encoder.
     */
    public void resetEncoders() {

        frontLeftEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backRightEncoder.setPosition(0);

    }

    /**
     * Drives the drivetrain with teleoperated controls.
     * @param leftJoystick The joystick controlling the left side of the drivetrain.
     * @param rightJoystick The joystick controlling the right side of the drivetrain.
     */
    public void drive(Joystick leftJoystick, Joystick rightJoystick) {

        frontLeft.set(Math.pow(leftJoystick.getRawAxis(1), 1));
        backLeft.set(Math.pow(leftJoystick.getRawAxis(1), 1));
        frontRight.set(Math.pow(rightJoystick.getRawAxis(1), 1));
        backRight.set(Math.pow(rightJoystick.getRawAxis(1),1));

    }

    /**
     * Drives the drivetrain autonomously in a straight line.
     * @param power The signed power to apply to the motor controllers to drive the drivetrain.
     */
    public void driveAutonomously(double power) {

        frontLeft.set(power);
        backLeft.set(power);
        frontRight.set(power);
        backRight.set(power);

    }

    /**
     * Drives the drivetrain autonomously a certain distance in a straight line.
     * @param power The signed power to apply to the motor controllers to drive the drivetrain.
     * @param distance The distance, in encoder units, to drive the drivetrain.
     * @param driverStationDisplay A boolean representing whether or not to display certain encoder values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain encoder values on SmartDashboard.
     */
    public void driveAutonomously(double power, double distance, boolean driverStationDisplay, boolean smartDashboardDisplay) {

        resetEncoders();

        while (Math.abs(frontLeftEncoder.getPosition()) < Math.abs(distance)) {
          
            frontLeft.set(power);
            backLeft.set(power);
            frontRight.set(power);
            backRight.set(power);
    
            if (driverStationDisplay) {

                DriverStation.reportWarning(Double.toString(frontLeftEncoder.getPosition()), false);
            
            }
        
            if (smartDashboardDisplay) {

                SmartDashboard.putNumber("Drive Encoder", frontLeftEncoder.getPosition());
            
            }
    
        }

        stop();

    }

    /**
     * Turns the drivetrain autonomously either left or right.
     * @param direction The String representing the direction ("LEFT" or "RIGHT") to turn the drivetrain.
     * @param power The unsigned power to apply to the motor controllers to turn the drivetrain.
     */
    public void turnAutonomously(String direction, double power) {

        if (direction.equals("LEFT")) {

            frontLeft.set(power);
            backLeft.set(power);
            frontRight.set(-power);
            backRight.set(-power);

        } else if (direction.equals("RIGHT")) {

            frontLeft.set(-power);
            backLeft.set(-power);
            frontRight.set(power);
            backRight.set(power);

        }

    }

    /**
     * Turns the drivetrain autonomously a certain angle either left or right.
     * @param direction A String representing the direction ("LEFT" or "RIGHT") to turn the drivetrain.
     * @param power The unsigned power to apply to the motor controllers to turn the drivetrain.
     * @param angle The angle, in gyroscope units, to turn the drivetrain
     * @param driverStationDisplay A boolean representing whether or not to display certain gyroscope values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain gyroscope values on SmartDashboard.
     */
    public void turnAutonomously(String direction, double power, double angle, boolean driverStationDisplay, boolean smartDashboardDisplay) {

    }

    /**
     * Stops the motor controllers.
     */
    public void stop() {

        frontLeft.set(0);
        backLeft.set(0);
        frontRight.set(0);
        backRight.set(0);

    }

    /**
     * Aims the robot at the target.
     * @param driverStationDisplay A boolean representing whether or not to display certain PID and Limelight values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID and Limelight values on SmartDashboard.
     * @param limelight The limelight attached to the drivetrain.
     */
    public void prepareToShoot(boolean driverStationDisplay, boolean smartDashboardDisplay, Limelight limelight) {

        limelight.updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);
    
        if (Math.abs(limelight.horizontalOffset) > 0.1) {
            
            if (limelight.horizontalOffset > 0) {

                turnAutonomously("LEFT", 0.1);

            }
            
            if (limelight.horizontalOffset < 0) {

                turnAutonomously("RIGHT", 0.1);

            }
    
        }
    
        stop();
    
    }
    
}
