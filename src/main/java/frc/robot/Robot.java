/*

    Notes:
    - The teleop driving might be reversed.
    - The autonomous code might not work.
    - The teleopPeriodic method has two calls to the prepareToShoot method, and both calls require the same button press.
        - It is possible that only one of these is necessary.
    - All calls to the prepareToShoot method utilize a full-power shooter.
        - This will be fixed upon creating a map of distance values to power values.
    - The stopMechanisms method might not work because of the Mechanism superclass.
    - At the end of the teleopPeriodic method, the shooter object might need to be included in the parameters of the stopMechanisms method.
    - I really don't like how the shooter object's initialization parameters are 44 and 55. Someone needs to fix that with the REV Hardware Client.

*/

package frc.robot;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
    // Spinner spinner = new Spinner(3);
    // Lift lift = new Lift(4, 5, 0, 1);
    Drivetrain drivetrain = new Drivetrain(6, 7, 8, 9);
    Shooter shooter = new Shooter(44, 55, 6e-5, 0, 0, 0, 0.000015, 1, -1, 6000);
    Limelight limelight = new Limelight();

    @Override
    public void robotInit() {

        compressor.clearAllPCMStickyFaults();
        compressor.start();

        intake.setInversionStatus(true);
        hopper.setInversionStatus(false, true);
        // spinner.setInversionStatus(false);
        // lift.setInversionStatus(true, false);
        drivetrain.setInversionStatus(false, false, true, true);

    }

    @Override
    public void autonomousInit() {

        // drivetrain.resetEncoders();
        // drivetrain.driveAutonomously(0.1, 35, false, true);

    }

    @Override
    public void autonomousPeriodic() {

        // intake.intake(0.1);
        // if (drivetrain.frontLeftEncoder.getPosition() < 35) {

        //     drivetrain.driveAutonomously(0);
        //     shooter.prepareToShoot(1, false, true, drivetrain, leftJoystick);
        //     drivetrain.driveAutonomously(0);

        // }

        // if (Math.abs(shooter.horizontalOffset) < 0.5) {

        //     hopper.moveUp(0.3);
            
        // }

    }

    @Override
    public void teleopInit() {

        drivetrain.driveAutonomously(0);
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

            drivetrain.prepareToShoot(false, true, leftJoystick, limelight);

        }

        // ??????????????????????????????
        if (leftJoystick.getRawButtonPressed(1)) {

            drivetrain.prepareToShoot(false, true, leftJoystick, limelight);

        }

        if (rightJoystick.getRawButton(1)) {

            hopper.moveUp(0.3);

        }

        // if (rightJoystick.getRawButton(2)) {

        //     spinner.freeSpin(0.3);

        // }

        // if (rightJoystick.getRawButton(3)) {
        
        //     spinner.beginRotationControl(true, false, leftJoystick);

        // }

        // if (rightJoystick.getRawButton(4)) {

        //     spinner.beginPositionControl(true, false, leftJoystick);

        // }

        // I have no idea which button works best for shooting. Fix later.


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

        // if (thirdJoystick.getPOV() == 270) {

        //     lift.freeRatchet(thirdJoystick);
        
        // }

        // if (thirdJoystick.getPOV() == 90) {

        //     lift.lockRatchet(thirdJoystick);
        
        // }

        // if (thirdJoystick.getPOV() == 0) {

        //     lift.moveUp(0.2);

        // }

        // if (thirdJoystick.getPOV() == 180) {

        //     lift.moveDown(0.2);

        // }

        if (leftJoystick.getRawButton(2)) {
            
            shooter.stop();

        }

        // if (thirdJoystick.getRawButton(3)) {

        //     try {

        //         drivetrain.startRecording(leftJoystick, rightJoystick, "C:\\Users\\FRC3408_User\\Downloads\\Class-Based Robot-20210130T183128Z-001\\Class-Based Robot-Imported\\src\\main\\java\\frc\\robot\\data.csv");

        //     } catch (Exception exception) {

        //         exception.printStackTrace();

        //     }

        // }

        // if (thirdJoystick.getRawButton(4)) {

        //     try {

        //         drivetrain.endRecording();

        //     } catch (Exception exception) {

        //         exception.printStackTrace();

        //     }

        // }

        // if (thirdJoystick.getRawButton(2)) {

        //     try {

        //         drivetrain.playRecording("C:\\Users\\FRC3408_User\\Downloads\\Class-Based Robot-20210130T183128Z-001\\Class-Based Robot-Imported\\src\\main\\java\\frc\\robot\\data.csv");

        //     } catch (Exception exception) {

        //         exception.printStackTrace();

        //     }

        // }

        // stopMechanisms(intake, hopper, spinner, lift);

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void disabledInit() {

        // stopMechanisms(intake, hopper, spinner, lift, drivetrain, shooter);
        stopMechanisms(intake, hopper, drivetrain, shooter);

    }

    public void stopMechanisms(Mechanism... args) {

        // for (Mechanism mechanism : args) {

        //     mechanism.stop();

        // }

    }

}