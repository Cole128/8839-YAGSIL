// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax iJoint;
  private CANSparkMax intake;
  private static Encoder iEncoder;
  public boolean intakeResting = true;
  
  /** Creates a new IntakeJoint. */
  public Intake() {
    iJoint = new CANSparkMax(MotorConstants.MOTOR_INTAKE_ANGLE_ID, MotorType.kBrushless);
    intake = new CANSparkMax(MotorConstants.MOTOR_INTAKE_ID, MotorType.kBrushless);

    iJoint.restoreFactoryDefaults();
    intake.restoreFactoryDefaults();

    iJoint.setSmartCurrentLimit(40);
    intake.setSmartCurrentLimit(40);
    
    iJoint.setIdleMode(IdleMode.kCoast);
    intake.setIdleMode(IdleMode.kBrake);

    iJoint.setInverted(false);
    intake.setInverted(true);

    iJoint.burnFlash();
    intake.burnFlash();

    iEncoder = new Encoder(0, 1);
  }

  public static double getEncoderDistance() {
    return iEncoder.getDistance();
  }

  @Override
  public void periodic() {
    if (intakeResting == true) {
      if (getEncoderDistance() < -1000) {
        iJoint.set(0.4);
      } else if (getEncoderDistance() < -500) {
        iJoint.set(0.3);
      }  else {
        iJoint.set(0);
      }
      
    } else if (intakeResting == false) {
      if (getEncoderDistance() > -1000) {
        iJoint.set(-0.6);
      } else if (getEncoderDistance() > -1500) {
        iJoint.set(-0.4);
      } else if (getEncoderDistance() > -1600) {
        iJoint.set(-0.3);
      } else {
        iJoint.set(0);
      }
      
    }
  }

  public void setIntakeOut() {
    intakeResting = false;
  }

  public void setIntakeIn() {
    intakeResting = true;
  }

  public void setJointSpeed(double speed) {
    iJoint.set(speed);
  }

  public void setIntakeSpeed(double speed) {
    intake.set(speed);
  }
}
