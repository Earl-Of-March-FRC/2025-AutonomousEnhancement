// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final TalonFX leftFront = new TalonFX(DrivetrainConstants.LEFT_FRONT_ID);
  private final TalonFX leftBack = new TalonFX(DrivetrainConstants.LEFT_BACK_ID);
  private final TalonFX rightFront = new TalonFX(DrivetrainConstants.RIGHT_FRONT_ID);
  private final TalonFX rightBack = new TalonFX(DrivetrainConstants.RIGHT_BACK_ID);

  private DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    leftBack.setControl(new Follower(leftFront.getDeviceID(), false));
    rightBack.setControl(new Follower(rightFront.getDeviceID(), false));

    MotorOutputConfigs brake = new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
    MotorOutputConfigs invertedBrake = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);

    leftFront.getConfigurator().apply(brake);
    leftBack.getConfigurator().apply(brake);
    rightFront.getConfigurator().apply(invertedBrake);
    rightBack.getConfigurator().apply(invertedBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain_Left", leftFront.get());
    SmartDashboard.putNumber("Drivetrain_Right", rightFront.get());
  }

  @Override
  public void simulationPeriodic() {}

  public void tankDrive(double leftSpeed,double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
    
    leftFront.getSimState().setRotorVelocity(leftSpeed);
    rightFront.getSimState().setRotorVelocity(rightSpeed);
  }
}
