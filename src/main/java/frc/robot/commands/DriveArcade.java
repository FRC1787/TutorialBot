// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveArcade extends CommandBase {
  /** Creates a new DriveArcade. */

  Drivetrain localDriveTrain;

  public DriveArcade(Drivetrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    localDriveTrain = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double linearSpeed = -RobotContainer.driverController.getRawAxis(Constants.DRIVER_CONTROLLER_LINEAR_AXIS);
    double angularSpeed = RobotContainer.driverController.getRawAxis(Constants.DRIVER_CONTROLLER_ANGULAR_AXIS);
    localDriveTrain.arcadeDrive(linearSpeed, angularSpeed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    localDriveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
