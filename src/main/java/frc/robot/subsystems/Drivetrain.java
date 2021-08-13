// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Drivetrain extends SubsystemBase {
  
  //spark max motor controllers
  /*
  public static CANSparkMax leftSpark1 = null;
  public static CANSparkMax leftSpark2 = null;
  public static CANSparkMax leftSpark3 = null;
  public static CANSparkMax rightSpark1 = null;
  public static CANSparkMax rightSpark2 = null;
  public static CANSparkMax rightSpark3 = null;*/

  public static SpeedController leftSpark1 = new CANSparkMax(Constants.left1, MotorType.kBrushless);
  public static SpeedController leftSpark2 = new CANSparkMax(Constants.left2, MotorType.kBrushless);
  public static SpeedController leftSpark3 = new CANSparkMax(Constants.left3, MotorType.kBrushless);
  public static SpeedController rightSpark1 = new CANSparkMax(Constants.right1, MotorType.kBrushless);
  public static SpeedController rightSpark2 = new CANSparkMax(Constants.right2, MotorType.kBrushless);
  public static SpeedController rightSpark3 = new CANSparkMax(Constants.right3, MotorType.kBrushless);
  private static double derivative;
  private static double proportional;
  private static double integral;
  private static double previousError;
  private static double pIDMotorVoltage;
  public static double angle;

  //encoders
  private CANEncoder leftEncoder = null;
  private CANEncoder rightEncoder = null;
  
  
  //navX
  public AHRS gyro = new AHRS();
  //public DifferentialDriveKinematics kDriveKinematics;// = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
  public DifferentialDriveOdometry odometry;// = new DifferentialDriveOdometry(gyro.getRotation2d());

  //motor groups + drive
  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;
  private DifferentialDrive drive;


  /** Creates a new Drivetrain. */
  public Drivetrain() {

    leftSpark1.setInverted(false);
    leftSpark2.setInverted(false);
    leftSpark3.setInverted(false);
    rightSpark1.setInverted(false);
    rightSpark2.setInverted(false);
    rightSpark3.setInverted(false);
    
    leftEncoder = ((CANSparkMax) leftSpark1).getEncoder();
    //private final Encoder rightEncoder = new Encoder(Constants.kRightEncoderPorts[0], Constants.kRightEncoderPorts[1]);
    rightEncoder = ((CANSparkMax) rightSpark1).getEncoder();

    leftMotors = new SpeedControllerGroup(leftSpark1, leftSpark2, leftSpark3);
    rightMotors = new SpeedControllerGroup(rightSpark1, rightSpark2, rightSpark3);
    drive = new DifferentialDrive(leftMotors, rightMotors);
    // Sets the distance per pulse for the encoders


    //leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    //rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    //kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    resetEncoders();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getRate());
    return new DifferentialDriveWheelSpeeds(leftDriveSpeed(), rightDriveSpeed());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void arcadeDrive(double linearSpeed, double angularSpeed) {
    drive.arcadeDrive(linearSpeed, angularSpeed);
  }

    //does this matter if we are arcade
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders() {
    //leftEncoder.reset();
    leftEncoder.setPosition(0);
    //rightEncoder.reset();
    rightEncoder.setPosition(0);
  }


  public double leftEncoderDistance() {
    return leftEncoder.getPosition() * (0.1524 * Math.PI) / 42 * 10.38;
  }

  public double rightEncoderDistance() {
    return -rightEncoder.getPosition() * (0.1524 * Math.PI) / 42 * 10.38;
  }

  public double leftDriveSpeed() {
    return (leftEncoder.getVelocity() / 60) * 42 * (0.1524 * Math.PI) / 42 * 10.38;
  }

  public double rightDriveSpeed() {
    return -(rightEncoder.getVelocity() / 60) * 42 * (0.1524 * Math.PI) / 42 * 10.38;
  }

  public double getAverageEncoderDistance() {
    //return (leftEncoder.getPosition() + rightEncoder.getDistance()) / 2.0;
    return (leftEncoderDistance() + rightEncoderDistance()) / 2.0;
  }

  public CANEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public CANEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();

    //this one might actually be right who knows
    //return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public double getError() {
    return proportional;
  }
  public double PIDDrive2(double targetDistance, double actualValue) {
    
    proportional = targetDistance - actualValue; // Math for determining motor output based on PID values
    derivative = (previousError - proportional) / 0.02; //every 0.02 seconds
    integral += previousError;
    previousError = proportional;

    /*return Math.abs(proportional) > okErrorRange ? // returns PID motor percentage
      Constants.maximumMotorOutput*Math.atan(proportional*10)*2/Math.PI
      : 0.0;*/
    /*return Math.abs(proportional) > okErrorRange ? // returns PID motor percentage
      truncate(Constants.PROPORTIONAL_TWEAK*proportional + Constants.DERIVATIVE_TWEAK*derivative + Constants.INTEGRAL_TWEAK*integral, Constants.maximumMotorOutput)
      : 0.0;*/
    
    if (Math.abs(proportional) > Constants.okErrorRange) // && !(targetDistance < actualValue && seekType == "oneWay")
    {
      pIDMotorVoltage = truncate((Constants.PROPORTIONAL_TWEAK * proportional)
          + (Constants.DERIVATIVE_TWEAK * derivative) + (Constants.INTEGRAL_TWEAK * integral), Constants.maximumMotorOutput);

      return pIDMotorVoltage;
    }

    return 0.0;

  }

  public double motorVoltage() {
    return pIDMotorVoltage;
  }

  private double truncate(double n, double maximum) { // return n if it's in range, if it's magnitude is large return maximum
    return Math.abs(n) > maximum ? maximum*Math.signum(n) : n;
  }

  private double angleFormat(double theta) { // in degrees lole
    return (theta + 180) % 360 - 180;
  }

  public void angleTurn(double setAngle) { // Gyro should be reset before method is used unless you're doing something weird
    tankDriveVolts(-PIDDrive2(setAngle, gyro.getYaw()), PIDDrive2(setAngle, gyro.getYaw()));
  }

  public void visionTurn() {
    gyro.reset();
    angle = truncate(angleFormat(gyro.getYaw()), 27);
    tankDriveVolts(PIDDrive2(0, Vision.lX), PIDDrive2(0, Vision.lX));
    gyro.reset();
  }

  public void visionFollow(double targetDistance) {
    tankDriveVolts(PIDDrive2(targetDistance, Vision.distanceToTarget()), -PIDDrive2(targetDistance, Vision.distanceToTarget()));
  }

  @Override
  public void periodic() {
    //odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getDistance());
    odometry.update(gyro.getRotation2d(), leftEncoderDistance(), rightEncoderDistance());
  }
}
