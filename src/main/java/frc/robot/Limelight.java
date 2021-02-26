package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends Mechanism {

    final double limelightHeight, targetHeight, limelightAngle;
    double horizontalOffset, verticalOffset, targetArea, targetValidity;

    NetworkTable limelightTable;
    NetworkTableEntry horizontalOffsetEntry, verticalOffsetEntry, targetAreaEntry, targetValidityEntry;

    public Limelight() {

        limelightHeight = 33.5;
        targetHeight = 97.0;
		limelightAngle = 3.0;
        
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        horizontalOffsetEntry = limelightTable.getEntry("tx");
        verticalOffsetEntry = limelightTable.getEntry("ty");
        targetAreaEntry = limelightTable.getEntry("ta");
        targetValidityEntry = limelightTable.getEntry("tv");

    }


    /**
     * Updates the Limelight variables and displays them on Driver Station and/or SmartDashboard if necessary.
     * @param driverStationDisplay A boolean representing whether or not to display certain PID values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID values on SmartDashboard.
     */
    public void updateLimelightVariables(boolean driverStationDisplay, boolean smartDashboardDisplay) {

        horizontalOffset = horizontalOffsetEntry.getDouble(0.0);
        verticalOffset = verticalOffsetEntry.getDouble(0.0);
        targetArea = targetAreaEntry.getDouble(0.0);
        targetValidity = targetValidityEntry.getDouble(0.0);    

        if (driverStationDisplay) {

            DriverStation.reportWarning(Double.toString(targetValidity), false);

        }

        if (smartDashboardDisplay) {

            SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
            SmartDashboard.putNumber("Vertical Offset", verticalOffset);
            SmartDashboard.putNumber("Target Area", targetArea);
            SmartDashboard.putNumber("Target Validity", targetValidity);

        }

    }
    
}
