/*

    Notes:
    - To Do: Research SmartDasboard

*/


package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends Mechanism {

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
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID values on Driver Station.
     */
    public void updatePIDCoefficients(boolean smartDashboardDisplay) {
    
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 1);
        double min = SmartDashboard.getNumber("Min Output", -1);
        double target = SmartDashboard.getNumber("SetPoint", 5000);
    
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

        if (target != kMaxRPM) {

            kMaxRPM = target;

        }
    
        shooterPID.setReference(kMaxRPM, ControlType.kVelocity);
        // shooterPID.setReference(5000, ControlType.kVelocity);

        if (smartDashboardDisplay) {
            
            SmartDashboard.putNumber("P Gain", kP);
            SmartDashboard.putNumber("I Gain", kI);
            SmartDashboard.putNumber("D Gain", kD);
            SmartDashboard.putNumber("I Zone", kIz);
            SmartDashboard.putNumber("Feed Forward", kFF);
            SmartDashboard.putNumber("Max Output", kMaxOutput);
            SmartDashboard.putNumber("Min Output", kMinOutput);
            SmartDashboard.putNumber("SetPoint", kMaxRPM);
            SmartDashboard.putNumber("ProcessVariable", leftEncoder.getVelocity());

        }

    }

    public void setPIDCoefficients(boolean smartDashboardDisplay, double ff, double p, double i, double d, double rpm) {

        if (smartDashboardDisplay) {
            
            SmartDashboard.putNumber("P Gain", p);
            SmartDashboard.putNumber("I Gain", i);
            SmartDashboard.putNumber("D Gain", d);
            SmartDashboard.putNumber("Feed Forward", ff);
            SmartDashboard.putNumber("SetPoint", rpm);

        }

        shooterPID.setFF(ff); 
        shooterPID.setP(p);
        shooterPID.setI(i);
        shooterPID.setD(d);
        shooterPID.setReference(rpm, ControlType.kVelocity);


    }

    /**
     * Spins the shooter using PID control.
     * @param smartDashboardDisplay A boolean representing whether or not to display certain PID values on Driver Station.
     */
    public void shoot(boolean smartDashboardDisplay) {

        updatePIDCoefficients(smartDashboardDisplay);

    }

    /**
     * Stops the shooter.
     */
    public void stop() {

        left.set(0);

    }

}