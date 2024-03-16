// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopShooter extends Command {
  private final Shooter shooterSubsystem;
  private final Intake intake;

  /** Creates a new SpinShooter. */
  public StopShooter(Shooter sSub, Intake iSub) {
    intake = iSub;
    shooterSubsystem = sSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sSub);
    addRequirements(iSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    shooterSubsystem.spinFeedandFlywheel(0, 0, 0);
    intake.setIntakeSpeed(0);
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.spinFeedandFlywheel(0, 0, 0);
    intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
