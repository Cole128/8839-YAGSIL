// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.climb.ClimbExtend;
import frc.robot.commands.climb.ClimbRetract;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.IntakeOut;
//import frc.robot.commands.shooter.ActuallyShoot;
import frc.robot.commands.shooter.FeedShooter;
import frc.robot.commands.shooter.HoldNote;
import frc.robot.commands.shooter.PrepareShooter;
import frc.robot.commands.shooter.ShooterOut;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.swervedrive.auto.DriveBack;
import frc.robot.commands.swervedrive.auto.DriveCurved;
import frc.robot.commands.swervedrive.auto.DriveCurvedRed;
import frc.robot.commands.swervedrive.auto.DriveForward;
import frc.robot.commands.swervedrive.auto.DriveReversedCurved;
import frc.robot.commands.swervedrive.auto.DriveReversedCurvedRed;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    public static final XboxController driverController = new XboxController(OperatorConstants.ControllerPort);
    public static final XboxController secondController = new XboxController(1);
    
    
        // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    public static Shooter shooterSubsystem = new Shooter();
    public static Elevator elevatorSubsystem = new Elevator();
    public static Intake intake = new Intake();

    //Shooter
    public static SpinShooter spinShooter = new SpinShooter(shooterSubsystem, intake);
    public static ShooterOut shooterOut = new ShooterOut(shooterSubsystem);
    public static FeedShooter feedShooter = new FeedShooter(intake);
    public static ShooterRev shooterRev = new ShooterRev(shooterSubsystem);
    public static PrepareShooter prepShooter = new PrepareShooter(shooterSubsystem);
    public static HoldNote holdNote = new HoldNote(shooterSubsystem, intake);
    //public static ActuallyShoot actuallyShoot = new ActuallyShoot();

    
    //Intake
    public static IntakeOut intakeOut = new IntakeOut(intake);
    public static IntakeNote intakeNote = new IntakeNote(intake);
    
    //Climb
    public static ClimbExtend climbExtend = new ClimbExtend(elevatorSubsystem);
    public static ClimbRetract climbRetract = new ClimbRetract(elevatorSubsystem);

    

    
    
            



    

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        NamedCommands.registerCommand("climbRetract", new ClimbRetract(elevatorSubsystem));
        NamedCommands.registerCommand("shoot", new SpinShooter(shooterSubsystem, intake));
        NamedCommands.registerCommand("rev", new ShooterRev(shooterSubsystem));
        NamedCommands.registerCommand("driveForward", new DriveForward(drivebase));
        NamedCommands.registerCommand("driveCurved", new DriveCurved(drivebase));
        NamedCommands.registerCommand("intakeOut", new IntakeOut(intake));
        NamedCommands.registerCommand("driveBack", new DriveBack(drivebase));
        NamedCommands.registerCommand("driveReversedCurved", new DriveReversedCurved(drivebase));
        NamedCommands.registerCommand("intakeNote", new IntakeNote(intake));


        NamedCommands.registerCommand("driveCurvedRed", new DriveCurvedRed(drivebase));
        NamedCommands.registerCommand("driveReversedCurvedRed", new DriveReversedCurvedRed(drivebase));


        // Configure the trigger bindings
        configureBindings();

        // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
        //         // Applies deadbands and inverts controls because joysticks
        //         // are back-right positive while robot
        //         // controls are front-left positive
        //         () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        //                 OperatorConstants.LEFT_Y_DEADBAND),
        //         () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        //                 OperatorConstants.LEFT_X_DEADBAND),
        //         () -> -driverXbox.getRightX(),
        //         () -> -driverXbox.getRightY());

        // AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
        //         () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        //                 OperatorConstants.LEFT_Y_DEADBAND),
        //         () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        //                 OperatorConstants.LEFT_X_DEADBAND),
        //         () -> driverXbox.getRawAxis(2));

        // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        //         () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
        //                 OperatorConstants.LEFT_Y_DEADBAND),
        //         () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
        //                 OperatorConstants.LEFT_X_DEADBAND),
        //         () -> MathUtil.applyDeadband(driverXbox.getRightX(),
        //                 OperatorConstants.RIGHT_X_DEADBAND),
        //         driverXbox::getYButtonPressed,
        //         driverXbox::getAButtonPressed,
        //         driverXbox::getXButtonPressed,
        //         driverXbox::getBButtonPressed);

        Supplier<Translation2d> movement = () -> {
            double rawX = -driverController.getRawAxis(1);
            double rawY = -driverController.getRawAxis(0);

            double newX = rawX * Math.sqrt(1 - ((rawY * rawY) / 2));
            double newY = rawY * Math.sqrt(1 - ((rawX * rawX / 2)));

            double vX = MathUtil.applyDeadband(newX, OperatorConstants.LEFT_X_DEADBAND);
            double vY = MathUtil.applyDeadband(newY, OperatorConstants.LEFT_Y_DEADBAND);
            return new Translation2d(vX, vY);
        };

        DoubleSupplier omega = () -> {
            return MathUtil.applyDeadband(driverController.getRawAxis(4), OperatorConstants.Z_DEADBAND);
        };
        
        TeleopDrive closedFieldRel = new TeleopDrive(drivebase, movement, omega, () -> true);

        // m_chooser.addOption("Closed Absolute Drive", closedAbsoluteDrive);
        // m_chooser.addOption("Closed Field Absolute Drive", closedFieldAbsoluteDrive);
        // m_chooser.addOption("Closed Absolute Drive Adv", closedAbsoluteDriveAdv);
        m_chooser.setDefaultOption("TeleOp", closedFieldRel);

        SmartDashboard.putData(m_chooser);


                

        // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive :
        // closedFieldAbsoluteDrive);
        drivebase.setDefaultCommand(m_chooser.getSelected());

        autoChooser = AutoBuilder.buildAutoChooser();

        autoChooser.addOption("Blue Side", new PathPlannerAuto("BlueAuto"));
        autoChooser.addOption("Red Side", new PathPlannerAuto("RedAuto"));
        autoChooser.addOption("Red Bottom Trap", new PathPlannerAuto("BlueAutoTrap"));
        autoChooser.addOption("Red Trap", new PathPlannerAuto("RedAutoTrap"));
        autoChooser.addOption("Shoot Still", new PathPlannerAuto("ShootStill"));
        autoChooser.addOption("Nothing", new InstantCommand());
        autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi"));

        autoChooser.setDefaultOption("BlueAuto", new PathPlannerAuto("FollowPath"));

        SmartDashboard.putData(autoChooser);
        
       
       
        
        //intakeJoint.setDefaultCommand(holdIntakeJoint);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        
        //feedShooterButton.whileTrue(feedShooter);
        //intakeNoteButton.whileTrue(intakeNote);
        //revShooter.onTrue(shooterRev);
        // extendElevatorButton.whileTrue(climbExtend);
        // retractElevatorButton.whileTrue(climbRetract);
        //extendIntakeJoint.whileTrue(intakeOut);
        //shooterOutButton.whileTrue(shooterOut);
        
        // extendIntakeJoint.onTrue(new ExtendIntakeJoint(Constants.JointConstants.kUpperLimit, intakeJoint));
        // retractIntakeJoint.onTrue(new RetractIntakeJoint(Constants.JointConstants.kLowerLimit, intakeJoint));
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        //driverController.button(2).onTrue(new InstantCommand(drivebase::zeroGyro));

        new JoystickButton(secondController, 1).whileTrue(shooterRev);
        new JoystickButton(driverController, 1).whileTrue(intakeOut);
        new JoystickButton(driverController, 2).onTrue((new InstantCommand(drivebase::zeroGyro)));
        new JoystickButton(driverController, 5).onTrue((new InstantCommand(elevatorSubsystem::toggleEl)));
        new JoystickButton(secondController, 3).onTrue((new InstantCommand(shooterSubsystem::toggleShooter)));
        new JoystickButton(secondController, 2).whileTrue(spinShooter);
        new JoystickButton(secondController, 7).whileTrue(intakeNote);
        new JoystickButton(secondController, 6).whileTrue(holdNote);       // new JoystickButton(driverController, 4).onTrue(actuallyShoot);

        
        
        
        //new JoystickButton(driverController, 6).toggleOnTrue((new InstantCommand(elevatorSubsystem::setElOut)));
        //new JoystickButton(driverController, 5).toggleOnTrue((new InstantCommand(elevatorSubsystem::setElIn)));
        // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
        // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}