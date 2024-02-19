// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
    private final CANSparkMax intake;

  public IntakeSubsystem(int intakeID) {
    intake = new CANSparkMax(intakeID, MotorType.kBrushless);
    intake.restoreFactoryDefaults();

    intake.setSmartCurrentLimit(30);
    intake.setIdleMode(IdleMode.kBrake);

    intake.setInverted(false);
    intake.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinIntakeWheel(double speed) {
    intake.set(speed);
  }
}
