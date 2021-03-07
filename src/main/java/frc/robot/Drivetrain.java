/*

    Notes:
    - The second turnAutonomous method is empty because of a lack of gyroscope code.

*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import java.io.File;
import java.io.FileWriter;
// import java.io.IOException;
import java.util.Scanner;


public class Drivetrain extends Mechanism {

    CANSparkMax frontLeft, backLeft, frontRight, backRight;
    CANEncoder frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder;

    String recordingStorage, playbackStorage;
    FileWriter writer;
    Scanner scanner;

    long recordingStartTime, playbackStartTime;
    boolean recordingHasStarted, playbackHasStarted;
    boolean onTime;
    double nextDouble;

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

        recordingHasStarted = false;
        playbackHasStarted = false;
        onTime = true;

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

        frontLeft.set(Math.pow(leftJoystick.getRawAxis(1), 3));
        backLeft.set(Math.pow(leftJoystick.getRawAxis(1), 3));
        frontRight.set(Math.pow(rightJoystick.getRawAxis(1), 3));
        backRight.set(Math.pow(rightJoystick.getRawAxis(1), 3));

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

    // /**
    //  * Starts recording values of the vertical axes of two joysticks, along with timestamps, in a csv file located at the specified file path.
    //  * @param leftJoystick The joystick controlling the left side of the drivetrain.
    //  * @param rightJoystick The joystick controlling the right side of the drivetrain.
    //  * @param filePath The String representing the location of the csv file to store the recorded values in.
    //  * @throws IOException
    //  */
    // public void startRecording(Joystick leftJoystick, Joystick rightJoystick, String filePath) throws IOException {

    //     recordingStorage = filePath;

    //     if (!recordingHasStarted) {

    //         try {

    //             writer = new FileWriter(recordingStorage);
    
    //         } catch (Exception exception) {
    
    //             exception.printStackTrace();
    
    //         }

    //         recordingStartTime = System.currentTimeMillis();
    //         recordingHasStarted = true;

    //     }

    //     if (writer != null) {

    //         writer.append("" + (System.currentTimeMillis() - recordingStartTime));
    //         writer.append("," + leftJoystick.getRawAxis(1));
    //         writer.append("," + rightJoystick.getRawAxis(1) + "\n");

    //     }

    // }

    // /**
    //  * Ends the latest recording.
    //  * @throws IOException
    //  */
    // public void endRecording() throws IOException {

    //     if (writer != null) {

    //         writer.flush();
    //         writer.close();

    //         recordingHasStarted = false;

    //         driveAutonomously(0);

    //     }

    // }

    // /**
    //  * Starts playing back the recording at the specified file path.
    //  * @param filePath The String representing the location of the csv file to play back.
    //  * @throws IOException
    //  */
    // public void playRecording(String filePath) throws IOException {

    //     playbackStorage = filePath;

    //     double leftStick, rightStick;
    //     double deltaT = 0;

    //     if (!playbackHasStarted) {

    //         try {

    //             scanner = new Scanner(new File(playbackStorage));
    //             scanner.useDelimiter(",|\\n");
    
    //         } catch (Exception exception) {
    
    //             exception.printStackTrace();
    
    //         }

    //         playbackStartTime = System.currentTimeMillis();
    //         playbackHasStarted = true;

    //     }
        
    //     if (scanner != null && scanner.hasNext()) {

    //         if (onTime) {

    //             nextDouble = scanner.nextDouble();

    //         }

    //         deltaT = nextDouble - (System.currentTimeMillis() - playbackStartTime);

    //         if (deltaT <= 0) {

    //             leftStick = scanner.nextDouble();
    //             rightStick = scanner.nextDouble();

    //             frontLeft.set(Math.pow(leftStick, 3));
    //             backLeft.set(Math.pow(leftStick, 3));
    //             frontRight.set(Math.pow(rightStick, 3));
    //             backRight.set(Math.pow(rightStick, 3));

    //             onTime = true;

    //         } else {

    //             onTime = false;

    //         }

    //     } else {

    //         scanner.close();
    //         scanner = null;

    //         playbackHasStarted = false;

    //         driveAutonomously(0);

    //     }

    // }

    /**
     * Aims the robot at the target.
     * @param driverStationDisplay A boolean representing whether or not to display certain PID and Limelight values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID and Limelight values on SmartDashboard.
     * @param joystick The joystick operating the shooter whose second button can be used to break loops.
     * @param limelight The limelight attached to the drivetrain.
     */
    public void prepareToShoot(boolean driverStationDisplay, boolean smartDashboardDisplay, Joystick joystick, Limelight limelight) {

        // Not sure if this line is strictly necessary.
        limelight.updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);
    
        // Might want to change this to an if statement and remove the joystick param.
        while (Math.abs(limelight.horizontalOffset) > 0.1) {
    
            limelight.updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);
        
            if (limelight.horizontalOffset > 0) {

                turnAutonomously("LEFT", 0.1);

            }
            
            if (limelight.horizontalOffset < 0) {

                turnAutonomously("RIGHT", 0.1);

            }
        
            if (joystick.getRawButton(2)) {

                stop();
                break;

            }
    
        }
    
        stop();
    
    }
    
}