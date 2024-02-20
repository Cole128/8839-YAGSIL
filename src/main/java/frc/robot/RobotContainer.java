// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbExtend;
import frc.robot.commands.ClimbRetract;
import frc.robot.commands.RetractIntake;

// import frc.robot.commands.ExtendIntakeJoint;
// import frc.robot.commands.HoldIntakeJoint;
//import frc.robot.commands.JointTest;
// import frc.robot.commands.RetractIntakeJoint;

import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.StopIntake;
// import frc.robot.commands.getIntakePos;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeJoint;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

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

    public static final JoystickButton retractIntakeJoint = new JoystickButton(driverController, Constants.Buttons.RETRACT_INTAKE_JOINT_BUTTON_ID);
    public static final JoystickButton extendIntakeJoint = new JoystickButton(driverController, Constants.Buttons.EXTEND_INTAKE_JOINT_BUTTON_ID);
    public static final JoystickButton spinIntakeButton = new JoystickButton(driverController, Constants.Buttons.INTAKE_BUTTON_ID);
    public static final JoystickButton spinShooterButton = new JoystickButton(driverController, Constants.Buttons.SHOOTER_BUTTON_ID);
    public static final JoystickButton extendElevatorButton = new JoystickButton(driverController, Constants.Buttons.ELEVATOR_EXTEND_BUTTON_ID);
    public static final JoystickButton retractElevatorButton = new JoystickButton(driverController, Constants.Buttons.ELEVATOR_RETRACT_BUTTON_ID);
    
        // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));
    public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem(MotorConstants.MOTOR_FLYWHEEL_LEFT_ID, MotorConstants.MOTOR_FLYWHEEL_RIGHT_ID, MotorConstants.MOTOR_FEEDER_ID);
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem(MotorConstants.MOTOR_INTAKE_ID);
    public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(MotorConstants.MOTOR_ELEVATOR_LEFT_ID, MotorConstants.MOTOR_ELEVATOR_RIGHT_ID);
    public static IntakeJoint intakeJoint = new IntakeJoint(MotorConstants.MOTOR_INTAKE_ANGLE_ID);

    //Shooter
    public static SpinShooter spinShooter = new SpinShooter(shooterSubsystem);

    
    //Intake
    public static SpinIntake spinIntake = new SpinIntake(intakeSubsystem);
    public static RetractIntake retractIntake = new RetractIntake(intakeJoint);
 
    
    //Climb
    public static ClimbExtend climbExtend = new ClimbExtend(elevatorSubsystem);
    public static ClimbRetract climbRetract = new ClimbRetract(elevatorSubsystem);

    
    
            



    

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        NamedCommands.registerCommand("climbRetract", new ClimbRetract(elevatorSubsystem));

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
        
        spinIntakeButton.whileTrue(spinIntake);
        spinShooterButton.whileTrue(spinShooter);
        extendElevatorButton.whileTrue(climbExtend);
        retractElevatorButton.whileTrue(climbRetract);
        retractIntakeJoint.onTrue(new RetractIntake(intakeJoint)).onFalse(new StopIntake(intakeJoint));
        
        // extendIntakeJoint.onTrue(new ExtendIntakeJoint(Constants.JointConstants.kUpperLimit, intakeJoint));
        // retractIntakeJoint.onTrue(new RetractIntakeJoint(Constants.JointConstants.kLowerLimit, intakeJoint));
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
        return null;
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}