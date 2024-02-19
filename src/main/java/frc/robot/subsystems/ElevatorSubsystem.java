// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax leftEl;
  private final CANSparkMax rightEl;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(int leftID, int rightID) {
    leftEl = new CANSparkMax(leftID, MotorType.kBrushless);
    rightEl = new CANSparkMax(rightID, MotorType.kBrushless);

    leftEl.restoreFactoryDefaults();
    rightEl.restoreFactoryDefaults();

    leftEl.setSmartCurrentLimit(30);
    rightEl.setSmartCurrentLimit(30);

    leftEl.setIdleMode(IdleMode.kBrake);
    rightEl.setIdleMode(IdleMode.kBrake);

    leftEl.setInverted(true);
    rightEl.setInverted(false);

    leftEl.burnFlash();
    rightEl.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climbExtend(double speed) {
    leftEl.set(speed);
    rightEl.set(speed);
  }
}
