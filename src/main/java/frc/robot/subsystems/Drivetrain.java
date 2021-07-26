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
  
  //spark motor controllers
  public static CANSparkMax leftSpark1 = new CANSparkMax(Constants.left1, MotorType.kBrushless);
  public static CANSparkMax leftSpark2 = new CANSparkMax(Constants.left2, MotorType.kBrushless);
  public static CANSparkMax leftSpark3 = new CANSparkMax(Constants.left3, MotorType.kBrushless);
  public static CANSparkMax rightSpark1 = new CANSparkMax(Constants.right1, MotorType.kBrushless);
  public static CANSparkMax rightSpark2 = new CANSparkMax(Constants.right2, MotorType.kBrushless);
  public static CANSparkMax rightSpark3 = new CANSparkMax(Constants.right3, MotorType.kBrushless);

  //encoders
  private static final CANEncoder leftEncoder = leftSpark1.getEncoder(EncoderType.kHallSensor, 42);
  private static final CANEncoder rightEncoder = rightSpark1.getEncoder(EncoderType.kHallSensor, 42);
  
  
  //navX
  public static AHRS gyro = new AHRS();
  //public DifferentialDriveKinematics kDriveKinematics;// = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
  public final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());// = new DifferentialDriveOdometry(gyro.getRotation2d());

  //motor groups + drive
  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;
  private DifferentialDrive drive;



  /** Creates a new Drivetrain. */
  public Drivetrain() {

    rightSpark1.setInverted(false);
    rightSpark2.setInverted(false);
    rightSpark3.setInverted(false);
    leftSpark1.setInverted(true);
    leftSpark2.setInverted(true);
    leftSpark3.setInverted(true);


    leftMotors = new SpeedControllerGroup(leftSpark1, leftSpark2, leftSpark3);
    rightMotors = new SpeedControllerGroup(rightSpark1, rightSpark2, rightSpark3);
    resetEncoders(); //originally at the end of this method
    drive = new DifferentialDrive(leftMotors, rightMotors);

    //kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
    

    setMaxOutput(0.3); //maybe go slow ~(=^･･^)_旦~ (ﾟoﾟ;)
  }

  @Override
  public void periodic() {
    //odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getDistance());
    odometry.update(gyro.getRotation2d(), leftEncoderDistance(), rightEncoderDistance());
  }
    
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDriveSpeed(), rightDriveSpeed());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void arcadeDrive(double linearSpeed, double angularSpeed) {
    drive.arcadeDrive(linearSpeed, angularSpeed);
    drive.feed();
  }

  //does this matter if we are arcade
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts); //originally only this was negative
    drive.feed();
  }

  public void resetEncoders() {
    //leftEncoder.reset();
    leftEncoder.setPosition(0);
    //rightEncoder.reset();
    rightEncoder.setPosition(0);
  }

  public double leftEncoderDistance() {
    return (leftEncoder.getPosition() / 10.38) * 0.479;
  }

  public double rightEncoderDistance() {
    return (-rightEncoder.getPosition() / 10.38) * 0.479;
  }

  public static double leftDriveSpeed() {
    return (leftEncoder.getVelocity() / 60) / 10.38 * (2*Math.PI) * 0.0762;
    //return ((leftEncoder.getVelocity() / 60)  / 10.38) * 0.479;
  }

  public static double rightDriveSpeed() {
    return (rightEncoder.getVelocity() / 60) / 10.38 * (2*Math.PI) * 0.0762;
    //return ((rightEncoder.getVelocity() / 60)  / 10.38) * 0.479;
  }

  public double getAverageEncoderDistance() {
    //return (leftEncoder.getPosition() + rightEncoder.getDistance()) / 2.0;
    return (leftEncoderDistance() + rightEncoderDistance()) / 2.0;
  }

  public static CANEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public static CANEncoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public static double getHeading() {
    return gyro.getRotation2d().getDegrees();

    //this one might actually be right who knows
    //return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public double getTurnRate() {
    return -gyro.getRate(); //used to be negative
  }


}
