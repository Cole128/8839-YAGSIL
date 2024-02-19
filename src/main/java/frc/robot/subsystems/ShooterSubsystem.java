// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax leftFly;
  private final CANSparkMax rightFly;
  private final CANSparkMax feed;

  public ShooterSubsystem(int leftFlyID, int rightFlyID, int feedID) {
    leftFly = new CANSparkMax(leftFlyID, MotorType.kBrushless);
    rightFly = new CANSparkMax(rightFlyID, MotorType.kBrushless);
    feed = new CANSparkMax(feedID, MotorType.kBrushless);

    leftFly.restoreFactoryDefaults();
    rightFly.restoreFactoryDefaults();
    feed.restoreFactoryDefaults();

    leftFly.setSmartCurrentLimit(30);
    rightFly.setSmartCurrentLimit(30);
    feed.setSmartCurrentLimit(30);

    leftFly.setIdleMode(IdleMode.kCoast);
    rightFly.setIdleMode(IdleMode.kCoast);
    feed.setIdleMode(IdleMode.kBrake);

    leftFly.setInverted(false);
    rightFly.setInverted(true);
    feed.setInverted(false);
    
    leftFly.burnFlash();
    rightFly.burnFlash();
    feed.burnFlash();
  }

  public void spinFeedWheel(double speed) {
    feed.set(speed);
  }

  public void spinFlywheel(double lSpeed, double rSpeed) {
    leftFly.set(lSpeed);
    rightFly.set(rSpeed);
  }

  public void spinFeedandFlywheel(double lSpeed, double rSpeed, double fSpeed) {
    leftFly.set(lSpeed);
    rightFly.set(rSpeed);
    feed.set(fSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
