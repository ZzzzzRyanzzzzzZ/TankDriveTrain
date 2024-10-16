// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drivetrain.DrivetrainIO;

import frc.robot.Subsystems.Drivetrain.DrivetrainIOInputsAutoLogged;
import frc.robot.Subsystems.Drivetrain.DrivetrainIOSim;

public class DrivetrainSubsystem extends SubsystemBase 
{
    public Command setVoltagesArcadesCommand(DoubleSupplier drive, DoubleSupplier steer) 
      {
        return new RunCommand(() -> {
        var speeds = DifferentialDrive.arcadeDriveIK(drive.getAsDouble(), steer.getAsDouble(), false);
        this.setVoltages(speeds.left * 12, speeds.right * 12);
    },  this);
}
  
  DrivetrainIO io = new DrivetrainIOSim();
  DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);


 // voltage control
  private void setVoltages(double left, double right) 
  {
    io.setVolts(left, right);

  }

/** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {}




  @Override
  public void periodic() {
  io.updateInputs(inputs);
  Logger.processInputs("Drivetrain", inputs);
  
  odometry.update(
      odometry.getPoseMeters().getRotation()
          // Use differential drive kinematics to find the rotation rate based on the wheel speeds and distance between wheels
          .plus(Rotation2d.fromRadians((inputs.leftVelocityMetersPerSecond - inputs.rightVelocityMetersPerSecond)
              * 0.020 / Units.inchesToMeters(26))),
      inputs.leftPositionMeters, inputs.rightPositionMeters);
      Logger.recordOutput("Drivebase Pose", odometry.getPoseMeters());
  }
  
}
