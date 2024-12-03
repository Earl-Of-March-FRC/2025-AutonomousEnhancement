// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(ArmConstants.LEFT_ID);
  private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(ArmConstants.RIGHT_ID);

  private final Encoder encoder = new Encoder(ArmConstants.ENCODER_CHANNEL_A, ArmConstants.ENCODER_CHANNEL_B); // Positive = arm going up
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    leftMotor.follow(rightMotor);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    encoder.setDistancePerPulse(360.0/ArmConstants.ENCODER_PPR);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm_Speed", getArmSpeed());
    SmartDashboard.putNumber("Arm_Deg", getArmDegrees());
  }

  @Override
  public void simulationPeriodic() {
    encoderSim.setDistance(encoderSim.getDistance()+(getArmSpeed()));
  }

  public void setSpeed(double speed) {
    rightMotor.set(MathUtil.clamp(speed, -ArmConstants.MAX_MOTOR_SPEED, ArmConstants.MAX_MOTOR_SPEED));
    encoderSim.setDirection(speed > 0);
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public double getArmDegrees() {
    return encoder.getDistance() + ArmConstants.ENCODER_OFFSET;
  }

  public double getArmSpeed() {
    return rightMotor.get();
  }
}
