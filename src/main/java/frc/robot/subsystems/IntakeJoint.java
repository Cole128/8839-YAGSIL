// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeJoint extends SubsystemBase {
  private CANSparkMax joint;
  private static DutyCycleEncoder encoder;
  
  /** Creates a new IntakeJoint. */
  public IntakeJoint(int jointID) {
    joint = new CANSparkMax(jointID, MotorType.kBrushed);
    encoder = new DutyCycleEncoder(0);
    encoder.setDistancePerRotation(10.0);
    encoder.setPositionOffset(0.83);

  }

  public static double getEncoderDistance() {
    return encoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    joint.set(speed);
  }
}
