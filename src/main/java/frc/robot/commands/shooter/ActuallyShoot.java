// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Intake;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ActuallyShoot extends SequentialCommandGroup {
//   /** Creates a new ActuallyShoot. */
//   private Shooter shooter = new Shooter();
//   private Intake intake = new Intake();
//   public ActuallyShoot() {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
      
//     new PrepareShooter(shooter).withTimeout(2),
//     new FeedShooter(intake).withTimeout(2),
//     new StopShooter(shooter, intake)
//     // new PrepareShooter(shooter, 1).withTimeout(2),
//       // new ParallelCommandGroup(
//       //   new PrepareShooter(shooter, 1),
//       //   new FeedShooter(intake, -1)
//       // ).withTimeout(2),
//       // // new FeedShooter(intake, -1).withTimeout(2),
//       // new ParallelCommandGroup(
//       //   new PrepareShooter(shooter, 0),
//       //   new FeedShooter(intake, 0)
//       //)
//    );
//   }
// }
