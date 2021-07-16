// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //sparks
    public static final int left1 = 20;
    public static final int left2 = 1;
    public static final int left3 = 2;
    public static final int right1 = 13;
    public static final int right2 = 14;
    public static final int right3 = 15;

    // Joysticks
    public static final int DRIVER_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER_LINEAR_AXIS = 1;
    public static final int DRIVER_CONTROLLER_ANGULAR_AXIS = 0;
    
    public static final int drivetrainOverrideButton = 12; // buttosn
    public static final int visionTurnButton = 3;

    //characterization
    public static final double ksVolts = 0.0418;
    public static final double kvVoltSecondsPerMeter = 2.87;
    public static final double kaVoltSecondsSquaredPerMeter = 0.175;
    public static final double kPDriveVel = 1.87*1.5;
    public static final double kTrackwidthMeters = 0.552865;
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    //encoders
	public static final int[] kLeftEncoderPorts = new int[] {0, 1}; //MAKE 6 ENCODERS
	public static final int[] kRightEncoderPorts = new int[] {2, 3};
	public static final double kEncoderDistancePerPulse = (0.1524 * Math.PI) / 42 * 10.38; //most definitely not 4096 or 1024
	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    
    //PID VALUES
	public static final double PROPORTIONAL_TWEAK = 0.007; // 0.0065 0.0047
	public static final double INTEGRAL_TWEAK = 0.0001;
    public static final double DERIVATIVE_TWEAK = 0.0005;
    public static final double maximumMotorOutput = 0.2;
    
    
    /*
    //navX
    public static final double ENCODER_RPM_TO_MPS = Math.PI * 2 * Units.inchesToMeters(3.0) / 60 / 7.56; // probably the real gear ratio
    */
}
