/*

    Notes:
    - The updatePIDCoefficients method call is currently commented out.
        - Once uncommented, it probably will not work.

*/


package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends Mechanism {

    final double limelightHeight, targetHeight, limelightAngle;
    double horizontalOffset, verticalOffset, targetArea, targetValidity;

    NetworkTable limelightTable;
    NetworkTableEntry horizontalOffsetEntry, verticalOffsetEntry, targetAreaEntry, targetValidityEntry;

    CANSparkMax left, right;
    CANEncoder leftEncoder, rightEncoder;

    CANPIDController shooterPID;
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMaxRPM;

    /**
     * Constructs a Shooter object.
     * @param leftID The ID of the left motor controller controlling the shooter.
     * @param rightID The ID of the right motor controller controlling the shooter.
     * @param P The proportional constant of the PID controller.
     * @param I The integration constant of the PID controller.
     * @param D The derivative constant of the PID controller.
     * @param Iz The integration zone of the PID controller.
     * @param FF The feedforward value of the PID controller.
     * @param maxOutput The maximum output of the PID controller.
     * @param minOutput The minimum output of the PID controller.
     * @param maxRPM The maximum RPM of the flywheel.
     */
    public Shooter(int leftID, int rightID, double P, double I, double D, double Iz, double FF, double maxOutput, double minOutput, double maxRPM) {

        limelightHeight = 33.5;
        targetHeight = 97.0;
		limelightAngle = 3.0;
        
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        horizontalOffsetEntry = limelightTable.getEntry("tx");
        verticalOffsetEntry = limelightTable.getEntry("ty");
        targetAreaEntry = limelightTable.getEntry("ta");
        targetValidityEntry = limelightTable.getEntry("tv");

        left = new CANSparkMax(leftID, MotorType.kBrushless);
        right = new CANSparkMax(rightID, MotorType.kBrushless);
        right.follow(left, true);
        leftEncoder = left.getEncoder();
        rightEncoder = right.getEncoder();

        shooterPID = left.getPIDController();
        kP = P;
        kI = I;
        kD = D;
        kIz = Iz;
        kFF = FF;
        kMaxOutput = maxOutput;
        kMinOutput = minOutput;
        kMaxRPM = maxRPM;
        shooterPID.setP(P);  
        shooterPID.setI(I);
        shooterPID.setD(D);
        shooterPID.setIZone(Iz);
        shooterPID.setFF(FF);
        shooterPID.setOutputRange(minOutput, maxOutput);

    }
    
    /**
     * Updates the PID coefficients by reading them off of SmartDashboard.
     * @param power The power to set the PID controller to.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID values on Driver Station.
     */
    public void updatePIDCoefficients(boolean smartDashboardDisplay) {
    
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
    
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
    
        shooterPID.setReference(5000, ControlType.kVelocity);

        if (smartDashboardDisplay) {
            
            SmartDashboard.putNumber("P Gain", kP);
            SmartDashboard.putNumber("I Gain", kI);
            SmartDashboard.putNumber("D Gain", kD);
            SmartDashboard.putNumber("I Zone", kIz);
            SmartDashboard.putNumber("Feed Forward Gain", kFF);
            SmartDashboard.putNumber("Max Output", kMaxOutput);
            SmartDashboard.putNumber("Min Output", kMinOutput);
            SmartDashboard.putNumber("Max RPM", kMaxRPM);
            SmartDashboard.putNumber("ProcessVariable", leftEncoder.getVelocity());

        }

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

    /**
     * Aims the robot at the target and powers the shooter.
     * @param power The unsigned power to apply to the motor controllers to spin the flywheel.
     * @param driverStationDisplay A boolean representing whether or not to display certain PID and Limelight values on Driver Station.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID and Limelight values on SmartDashboard.
     * @param drivetrain The drivetrain attached to the shooter.
     * @param joystick The joystick operating the shooter whose second button can be used to break loops.
     */
    public void prepareToShoot(boolean driverStationDisplay, boolean smartDashboardDisplay, Drivetrain drivetrain, Joystick joystick) {

        updatePIDCoefficients(smartDashboardDisplay);

        updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);
            
        SmartDashboard.putNumber("RPM", leftEncoder.getVelocity());

    
        if (joystick.getRawButton(2)) {

            left.set(0);

        }
    
        if (Math.abs(horizontalOffset) > 0.1) {

            updatePIDCoefficients(smartDashboardDisplay);
    
            updateLimelightVariables(driverStationDisplay, smartDashboardDisplay);
        
            if (horizontalOffset > 0) {

                drivetrain.turnAutonomously("LEFT", 0.1);

            }
            
            if (horizontalOffset < 0) {

                drivetrain.turnAutonomously("RIGHT", 0.1);

            }
        
            if (joystick.getRawButton(2)) {

                drivetrain.stop();
            //     break;

            }
    
        }
    
        drivetrain.driveAutonomously(0);
    
    }

    public void stop() {

        left.set(0);

    }

}