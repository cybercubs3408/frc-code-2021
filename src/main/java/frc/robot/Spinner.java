/*

    Notes:
    - The beginRotationControl method might not work.
    - The beginPositionControl method might not work.

*/


package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


public class Spinner extends Mechanism {

    final Color kBlueTarget, kRedTarget, kGreenTarget, kYellowTarget;

    TalonSRX spinner;
    I2C.Port i2cPort;
    ColorSensorV3 colorSensor;
    ColorMatch colorMatcher;

    Color detectedColor;
    ColorMatchResult match;
    String colorString, gameDataString, wantedColorString;

    /**
     * Constructs a Spinner object. Initializes the four color objects (blue, red, green, and yellow), the color sensor, and the color matcher.
     * @param spinnerID The ID of the motor controller controlling the spinner.
     */
    public Spinner(int spinnerID) {

        kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
        
        spinner = new TalonSRX(spinnerID);
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
        colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kYellowTarget);

    }
    
    /**
     * Sets the inversion status of each motor controller.
     * @param spinnerInversion A boolean representing whether or not to invert the spinner motor controller.
     */
    public void setInversionStatus(boolean spinnerInversion) {

        spinner.setInverted(spinnerInversion);

    }
    
    /**
     * Spins the spinner.
     * @param power The signed power to apply to the motor controller to spin the spinner.
     */
    public void freeSpin(double power) {

        spinner.set(ControlMode.PercentOutput, power);

    }

    /**
     * Updates the color variables and displays them on Driver Station and/or SmartDashboard if necessary.<p>
     * The String colorString is a representation of the color currently seen by the robot's color sensor.<p>
     * The String gameDataString is a representation of the color the game's color sensor should see during POSITION CONTROL.<p>
     * The String wantedColorString is a representation of the color the robot's color sensor should see so that the game's color sensor sees the color represented by gameDataString.
     * @param driverStationDisplay A boolean representing whether or not to display certain color values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain color values on SmartDashboard.
     */
    public void updateColorVariables(boolean driverStationDisplay, boolean smartDashboardDisplay) {

        detectedColor = colorSensor.getColor();
        match = colorMatcher.matchClosestColor(detectedColor);
        gameDataString = DriverStation.getInstance().getGameSpecificMessage();

        colorString = match.color == kBlueTarget ? "B" : match.color == kRedTarget ? "R" : match.color == kGreenTarget ? "G" : match.color == kYellowTarget ? "Y" : "U";
        wantedColorString = gameDataString.equals("B") ? "R" : gameDataString.equals("R") ? "B" : gameDataString.equals("G") ? "Y" : gameDataString.equals("Y") ? "G" : "";

        if (driverStationDisplay) {

            DriverStation.reportWarning(colorString, false);

        }

        if (smartDashboardDisplay) {

            SmartDashboard.putString("Detected Color", colorString);
            SmartDashboard.putString("Wanted Color", wantedColorString);
            SmartDashboard.putString("Game Data", gameDataString);

        }

    }

    /**
     * Begins ROTATION CONTROL, which involves spinning the CONTROL PANEL greater than 3 but fewer than 5 revolutions.
     * @param driverStationDisplay A boolean representing whether or not to display certain color values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain color values on SmartDashboard.
     * @param joystick The joystick operating the spinner whose second button can be used to break loops.
     */
    public void beginRotationControl(boolean driverStationDisplay, boolean smartDashboardDisplay, Joystick joystick) {

        for (int i = 0; i < 16; i++) {

            updateColorVariables(driverStationDisplay, smartDashboardDisplay);

            if (joystick.getRawButton(2)) {

                freeSpin(0);
                break;

            }

            if (match.color == kRedTarget) {

                while (match.color != kBlueTarget) {

                    updateColorVariables(driverStationDisplay, smartDashboardDisplay);
                    freeSpin(0.6);

                    if (joystick.getRawButton(2)) {

                        freeSpin(0);
                        break;
                        
                    }

                }

            }

            else if (match.color == kBlueTarget) {

                while (match.color != kRedTarget) {

                    updateColorVariables(driverStationDisplay, smartDashboardDisplay);
                    freeSpin(0.6);

                    if (joystick.getRawButton(2)) {

                        freeSpin(0);
                        break;

                    }

                }

            }

            else {

                while (match.color != kRedTarget) {

                    updateColorVariables(driverStationDisplay, smartDashboardDisplay);
                    freeSpin(0.6);

                    if (joystick.getRawButton(2)) {

                        freeSpin(0);
                        break;

                    }

                }

            }

        }

        freeSpin(0);

    }

    /**
     * Begins POSITION CONTROL, which involves spinning the CONTROL PANEL until the game's color sensor aligns with a specified color.
     * @param driverStationDisplay A boolean representing whether or not to display certain color values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain color values on SmartDashboard.
     * @param joystick The joystick operating the spinner whose second button can be used to break loops.
     */
    public void beginPositionControl(boolean driverStationDisplay, boolean smartDashboardDisplay, Joystick joystick) {

        updateColorVariables(driverStationDisplay, smartDashboardDisplay);

        while (!colorString.equals(wantedColorString)) {

            updateColorVariables(driverStationDisplay, smartDashboardDisplay);
            freeSpin(0.6);

            if (joystick.getRawButton(2)) {

                freeSpin(0);
                break;

            }

        }

        freeSpin(0);

    }

    public void stop() {

        spinner.set(ControlMode.PercentOutput, 0);

    }

}