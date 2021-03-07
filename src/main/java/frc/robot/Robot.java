/*

    Notes:
    - The autonomous code might not work.
    - The teleopPeriodic method has two calls to the prepareToShoot method, and both calls require the same button press.
        - It is possible that only one of these is necessary.
    - The stopMechanisms method might not work because of the Mechanism superclass.
    - At the end of the teleopPeriodic method, the shooter object might need to be included in the parameters of the stopMechanisms method.
    - I really don't like how the shooter object's initialization parameters are 44 and 55. Someone needs to fix that with the REV Hardware Client.

*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends TimedRobot {

    Compressor compressor = new Compressor();

    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick thirdJoystick = new Joystick(2);

    Intake intake = new Intake(0, 2);
    Hopper hopper = new Hopper(1, 2);
    Drivetrain drivetrain = new Drivetrain(6, 7, 8, 9);
    Shooter shooter = new Shooter(44, 55, 6e-5, 0, 0, 0, 0.000015, 1, -1, 6000);
    Limelight limelight = new Limelight(33.5, 97.0, 3.0);

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

        drivetrain.drive(rightJoystick, leftJoystick);

        //if (thirdJoystick.getRawButton(1)) {

            intake.stop();
            hopper.stop();

        //}

        if (leftJoystick.getRawButton(1)) {

            drivetrain.prepareToShoot(false, true, limelight);

        }

        // ??????????????????????????????
        if (leftJoystick.getRawButtonPressed(1)) {

            drivetrain.prepareToShoot(false, true, limelight);

        }

        if (rightJoystick.getRawButton(1)) {

            hopper.moveUp(0.3);

        }


        if (rightJoystick.getRawButton(2)) {

            shooter.shoot(true);

        }

        if (thirdJoystick.getRawAxis(1) > 0.9) {

            intake.raiseArm();

        }

        if (thirdJoystick.getRawAxis(1) < -0.9) {

            intake.dropArm();

        }

        if (thirdJoystick.getRawAxis(2) > 0.9) {

            intake.outtake(0.5);

        }

        if (thirdJoystick.getRawAxis(3) > 0.9) {

            intake.intake(0.5);
        
        }

        if (thirdJoystick.getRawButton(1)) {

            hopper.moveUp(0.1);

        }

        if (thirdJoystick.getRawButton(5)) {

            hopper.moveDown(0.1);

        }

        if (thirdJoystick.getRawButton(6)) {

            hopper.moveUp(0.4);

        }

        if (thirdJoystick.getRawButton(4)) {
            
            hopper.moveUp(0.2);

        }

        if (leftJoystick.getRawButton(2)) {
            
            shooter.stop();

        }

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