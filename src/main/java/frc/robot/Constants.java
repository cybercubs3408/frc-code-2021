// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int frontLeftMotorID = 6;
        public static final int backLeftMotorID = 7;
        public static final int frontRightMotorID = 8;
        public static final int backRightMotorID = 9;

        public static final int[] leftEncoderIDs = new int[] {6, 7};
        public static final int[] rightEncoderIDs = new int[] {8, 9};

        public static final boolean leftEncoderReversed = false;
        public static final boolean rightEncoderReversed = true;

        public static final int encoderCPR = 1024;
        public static final double wheelDiameterInches = 6;
        public static final double encoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (wheelDiameterInches * Math.PI) / (double) encoderCPR;
    }

    public static final class IOConstants {
        public static final int leftJoystickID = 0;
        public static final int rightJoystickID = 1;
        public static final int xBoxControllerID = 2;
    }

}
