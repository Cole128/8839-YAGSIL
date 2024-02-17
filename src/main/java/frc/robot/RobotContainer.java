// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbExtend;
import frc.robot.commands.ClimbRetract;
import frc.robot.commands.SpinFeedWheel;
import frc.robot.commands.SpinFlywheel;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinShooter;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
//import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static final XboxController driverController = new XboxController(OperatorConstants.ControllerPort);

    public static final JoystickButton spinFlywheelButton = new JoystickButton(driverController, Constants.Buttons.FLYWHEEL_BUTTON_ID);
    public static final JoystickButton spinFeedWheelButton = new JoystickButton(driverController, Constants.Buttons.FEEDWHEEL_BUTTON_ID);
    public static final JoystickButton spinIntakeButton = new JoystickButton(driverController, Constants.Buttons.INTAKE_BUTTON_ID);
    public static final JoystickButton spinShooterButton = new JoystickButton(driverController, Constants.Buttons.SHOOTER_BUTTON_ID);
    public static final JoystickButton extendElevatorButton = new JoystickButton(driverController, Constants.Buttons.ELEVATOR_EXTEND_BUTTON_ID);
    public static final JoystickButton retractElevatorButton = new JoystickButton(driverController, Constants.Buttons.ELEVATOR_RETRACT_BUTTON_ID);
    
        // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    public static SpinFlywheel spinFlywheel = new SpinFlywheel(shooterSubsystem);
    public static SpinFeedWheel spinFeedWheel = new SpinFeedWheel(shooterSubsystem);
    public static SpinIntake spinIntake = new SpinIntake(intakeSubsystem);
    public static SpinShooter spinShooter = new SpinShooter(shooterSubsystem);
    public static ClimbExtend climbExtend = new ClimbExtend(elevatorSubsystem);
    public static ClimbRetract climbRetract = new ClimbRetract(elevatorSubsystem);
            



    

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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

        TeleopDrive closedFieldRel = new TeleopDrive(
                drivebase,
                () -> MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_X_DEADBAND) * 0.7,
                () -> MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_Y_DEADBAND) * 0.7,
                () -> MathUtil.applyDeadband(driverController.getRawAxis(4), OperatorConstants.Z_DEADBAND) * 0.8, () -> true);

        // m_chooser.addOption("Closed Absolute Drive", closedAbsoluteDrive);
        // m_chooser.addOption("Closed Field Absolute Drive", closedFieldAbsoluteDrive);
        // m_chooser.addOption("Closed Absolute Drive Adv", closedAbsoluteDriveAdv);
        m_chooser.setDefaultOption("TeleOp", closedFieldRel);

        SmartDashboard.putData(m_chooser);

        // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive :
        // closedFieldAbsoluteDrive);
        drivebase.setDefaultCommand(m_chooser.getSelected());
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
        spinFlywheelButton.whileTrue(spinFlywheel);
        spinFeedWheelButton.whileTrue(spinFeedWheel);
        spinIntakeButton.whileTrue(spinIntake);
        spinShooterButton.whileTrue(spinShooter);
        extendElevatorButton.whileTrue(climbExtend);
        retractElevatorButton.whileTrue(climbRetract);
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        //driverController.button(2).onTrue(new InstantCommand(drivebase::zeroGyro));

        // new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
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
        // return drivebase.getAutonomousCommand("New Path", true);
        return m_chooser.getSelected();
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}