/*

    Notes:

*/


package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;


public class Intake extends Mechanism {
    
    TalonSRX intakeT;
    Solenoid intakeArm;

    /**
     * Constructs an Intake object.
     * @param intakeID The ID of the motor controller controlling the intake.
     * @param intakeArmChannel The channel number on the PCM for the solenoid controlling the intake arm.
     */
    public Intake(int intakeID, int intakeArmChannel) {

        intakeT = new TalonSRX(intakeID);
        intakeArm = new Solenoid(intakeArmChannel);

    }

    /**
     * Sets the inversion status of each motor controller.
     * @param intakeInversion A boolean representing whether or not to invert the intake motor controller.
     */
    public void setInversionStatus(boolean intakeInversion) {

        intakeT.setInverted(intakeInversion);

    }

    /**
     * Raises the intake arm.
     */
    public void raiseArm() {

        intakeArm.set(true);

    }

    /**
     * Lowers the intake arm.
     */
    public void dropArm() {

        intakeArm.set(false);

    }

    /**
     * Spins the intake inward.
     * @param power The unsigned power to apply to the motor controller to spin the intake inward.
     */
    public void intake(double power) {

        intakeT.set(ControlMode.PercentOutput, Math.abs(power));

    }

    /**
     * Spins the intake outward.
     * @param power The unsigned power to apply to the motor controller to spin the intake outward.
     */
    public void outtake(double power) {

        intakeT.set(ControlMode.PercentOutput, -Math.abs(power));

    }

    /**
     * Stops the motor controller.
     */
    public void stop() {

        intakeT.set(ControlMode.PercentOutput, 0);

    }

}