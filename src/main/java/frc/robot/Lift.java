/*

    Notes:

*/


package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;


public class Lift extends Mechanism {

    TalonSRX left, right;
    DoubleSolenoid ratchet;

    /**
     * Constructs a Lift object.
     * @param leftID The ID of the left motor controller controlling the lift.
     * @param rightID The ID of the right motor controller controlling the lift.
     * @param ratchetForwardChannel The forward channel number on the PCM for the double solenoid controlling the ratchet.
     * @param ratchetReverseChannel The reverse channel number on the PCM for the double solenoid controlling the ratchet.
     */
    public Lift(int leftID, int rightID, int ratchetForwardChannel, int ratchetReverseChannel) {

        left = new TalonSRX(leftID);
        right = new TalonSRX(rightID);
        ratchet = new DoubleSolenoid(ratchetForwardChannel, ratchetReverseChannel);

    }

    /**
     * Sets the inversion status of each motor controller.
     * @param leftInversion A boolean representing whether or not to invert the left motor controller.
     * @param rightInversion A boolean representing whether or not to invert the right motor controller.
     */
    public void setInversionStatus(boolean leftInversion, boolean rightInversion) {

        left.setInverted(leftInversion);
        right.setInverted(rightInversion);

    }

    /**
     * Frees the ratchet and turns off the rumble in the specified joystick.
     * @param joystick The joystick operating the lift.
     */
    public void freeRatchet(Joystick joystick) {

        ratchet.set(Value.kReverse);
        joystick.setRumble(RumbleType.kLeftRumble, 0);
        joystick.setRumble(RumbleType.kRightRumble, 0);

    }  

    /**
     * Locks the ratchet and turns on the rumble in the specified joystick.
     * @param joystick The joystick operating the lift.
     */
    public void lockRatchet(Joystick joystick) {

        ratchet.set(Value.kForward);
        joystick.setRumble(RumbleType.kLeftRumble, 1);
        joystick.setRumble(RumbleType.kRightRumble, 1);

    }

    /**
     * Raises the lift.
     * @param power The unsigned power to apply to the motor controllers to raise the lift.
     */
    public void moveUp(double power) {

        left.set(ControlMode.PercentOutput, Math.abs(power));
        right.set(ControlMode.PercentOutput, Math.abs(power));

    }

    /**
     * Lowers the lift.
     * @param power The unsigned power to apply to the motor controllers to lower the lift.
     */
    public void moveDown(double power) {

        left.set(ControlMode.PercentOutput, -Math.abs(power));
        right.set(ControlMode.PercentOutput, -Math.abs(power));

    }

    /**
     * Stops the motor controllers.
     */
    public void stop() {

        left.set(ControlMode.PercentOutput, 0);
        right.set(ControlMode.PercentOutput, 0);
        
    }
    
}