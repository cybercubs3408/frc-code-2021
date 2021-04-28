/*

    Notes:
    - To Do: Fix issue of hopper not activating.

*/


package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Hopper extends Mechanism {

    TalonSRX left, right;

    /**
     * Constructs a Hopper object.
     * @param leftID The ID of the left motor controller controlling the hopper.
     * @param rightID The ID of the right motor controller controlling the hopper.
     */
    public Hopper(int leftID, int rightID) {

        left = new TalonSRX(leftID);
        right = new TalonSRX(rightID);

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
     * Moves the hopper up.
     * @param power The unsigned power to apply to the motor controllers to move the hopper up.
     */
    public void moveUp(double power) {

        left.set(ControlMode.PercentOutput, Math.abs(power));
        right.set(ControlMode.PercentOutput, Math.abs(power));

    }

    /**
     * Moves the hopper down.
     * @param power The unsigned power to apply to the motor controllers to move the hopper down.
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