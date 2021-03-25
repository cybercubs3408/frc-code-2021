/*

    Notes:
    - The autonomous code might not work.
    - The teleopPeriodic method has two calls to the prepareToShoot method, and both calls require the same button press.
        - It is possible that only one of these is necessary.
    - The stopMechanisms method might not work because of the Mechanism superclass.
    - At the end of the teleopPeriodic method, the shooter object might need to be included in the parameters of the stopMechanisms method.
    - I really don't like how the shooter object's initialization parameters are 44 and 55. Someone needs to fix that with the REV Hardware Client.
    - To Do: Name buttons appropriately.

*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Robot extends TimedRobot {

    boolean armRaised = false;
    boolean shooting = false;
    boolean drivingLocked = true;

    Compressor compressor = new Compressor();

    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick thirdJoystick = new Joystick(2);

    Intake intake = new Intake(0, 2);
    Hopper hopper = new Hopper(1, 2);
    Drivetrain drivetrain = new Drivetrain(6, 7, 8, 9);
    Shooter shooter = new Shooter(44, 55, 6e-5, 0, 0, 0, 0.000015, 1, -1, 6000);
    Limelight limelight = new Limelight(33.5, 97.0, 3.0);

    TalonSRX talon2 = new TalonSRX(2); /* Talon SRX on CAN bus with device ID 2*/
    PigeonIMU pigeon = new PigeonIMU(talon2); /* Pigeon is plugged into Talon 2*/

    @Override
    public void robotInit() {

        compressor.clearAllPCMStickyFaults();
        compressor.start();

        intake.setInversionStatus(true);
        hopper.setInversionStatus(false, true);
        drivetrain.setInversionStatus(false, false, true, true);

    }

    @Override
    public void autonomousInit() {


    }

    @Override
    public void autonomousPeriodic() {


    }

    @Override
    public void teleopInit() {

        drivetrain.stop();
        drivetrain.resetEncoders();

        CameraServer.getInstance().startAutomaticCapture(0);
        CameraServer.getInstance().startAutomaticCapture(1);

    }

    @Override
    public void teleopPeriodic() {

        drivetrain.drive(rightJoystick, leftJoystick, drivingLocked);

        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("Pigeon Yaw", ypr[0]);

        // System.out.println("Pigeon Yaw is: " + ypr[0]);

        if (shooting) {
            
            shooter.shoot(true);

        }

        //if (thirdJoystick.getRawButton(1)) {

            // Test if neccessary
            intake.stop();
            hopper.stop();

        //}

        if (leftJoystick.getRawButton(1)) {

            drivetrain.prepareToShoot(false, true, limelight);

        }

        // Test when testing prepareToShoot
        if (leftJoystick.getRawButtonPressed(1)) {

            drivetrain.prepareToShoot(false, true, limelight);

        }

        // May be obsolete.
        if (leftJoystick.getRawButton(2)) {
            
            shooting = false;
            shooter.stop();

        }

        if (leftJoystick.getRawButton(4)) {

            intake.intake(0.5);

        }

        if (rightJoystick.getRawButton(1)) {

            hopper.moveUp(0.4);

        }


        if (rightJoystick.getRawButton(2)) {

            if (shooting) {

                shooter.stop();

            }

            shooting = !shooting;

        }

        if (rightJoystick.getRawButton(4)) {
            
            drivingLocked = !drivingLocked;

        }


        if (thirdJoystick.getRawButtonPressed(1)) {

            if (armRaised) {
                
                intake.dropArm();

            } 
            
            else {
                
                intake.raiseArm();

            }

            armRaised = !armRaised;

        }

        if (thirdJoystick.getRawAxis(5) < -0.9) {

            intake.intake(0.5);
        
        }

        if (thirdJoystick.getRawAxis(5) > 0.9) {

            intake.outtake(0.5);

        }
        
        if (thirdJoystick.getRawAxis(1) < 0) {

            hopper.moveUp(0.5 * thirdJoystick.getRawAxis(1));

        }

        if (thirdJoystick.getRawAxis(1) > 0) {

            hopper.moveDown(0.5 * thirdJoystick.getRawAxis(1));

        }

        if (thirdJoystick.getRawButton(6)) {

            hopper.moveUp(0.5);

        }

        if (thirdJoystick.getRawButton(5)) {

            hopper.moveDown(0.5);

        }

        // if (leftJoystick.getRawButton(7)) {

        //     double [] ypr = new double[3];
        //     pigeon.GetYawPitchRoll(ypr);
        //     System.out.println("Yaw:" + ypr[0]);    

        // }

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void disabledInit() {

        // stopMechanisms(intake, hopper, drivetrain, shooter);

    }

    // Fix later.
    // public void stopMechanisms(Mechanism... args) {

    //     for (Mechanism mechanism : args) {

    //         mechanism.stop();

    //     }

    // }

}