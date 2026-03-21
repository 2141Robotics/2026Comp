// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.auto.ShootAtHubCommand;
import frc.robot.commands.auto.WiggleIntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.FieldMeasurements;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVE_CONTROLLER_PORT);

  final CommandXboxController operatorXbox = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));

  // private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Turret turret = new Turret(drivebase);
  private final Shooter shooter = new Shooter(drivebase);
  public final LEDs leds = new LEDs();

  private final SendableChooser<Command> autoChooser;

  DoubleSupplier leftX = () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND);
  DoubleSupplier leftY = () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.DEADBAND);

  DoubleSupplier rightX = () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.DEADBAND);

  DoubleSupplier rightY = () -> MathUtil.applyDeadband(driverXbox.getRightY(), OperatorConstants.DEADBAND);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.DEADBAND) * -1
          * (OperatorConstants.MIN_INPUTTED_SPEED +
              (OperatorConstants.NORMAL_INPUTTED_SPEED - OperatorConstants.MIN_INPUTTED_SPEED)
                  * (1 - driverXbox.getLeftTriggerAxis())
              +
              (OperatorConstants.MAX_INPUTTED_SPEED - OperatorConstants.NORMAL_INPUTTED_SPEED)
                  * driverXbox.getRightTriggerAxis()),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND) * -1
          * (OperatorConstants.MIN_INPUTTED_SPEED +
              (OperatorConstants.NORMAL_INPUTTED_SPEED - OperatorConstants.MIN_INPUTTED_SPEED)
                  * (1 - driverXbox.getLeftTriggerAxis())
              +
              (OperatorConstants.MAX_INPUTTED_SPEED - OperatorConstants.NORMAL_INPUTTED_SPEED)
                  * driverXbox.getRightTriggerAxis()))
      .withControllerRotationAxis(() -> rightX.getAsDouble())
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(rightX, rightY)
      .headingWhile(true);

  public double test() {
    return rightX.getAsDouble();
  }

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  // SwerveInputStream driveAngularVelocityKeyboard =
  // SwerveInputStream.of(drivebase.getSwerveDrive(),
  // () -> -driverXbox.getLeftY(),
  // () -> -driverXbox.getLeftX())
  // .withControllerRotationAxis(rightX)
  // .deadband(OperatorConstants.DEADBAND)
  // .scaleTranslation(OperatorConstants.DRIVER_CONTROLLER_JOYSTICK_SCALER)
  // .allianceRelativeControl(true);
  // // Derive the heading axis with math!
  // SwerveInputStream driveDirectAngleKeyboard =
  // driveAngularVelocityKeyboard.copy()
  // .withControllerHeadingAxis(() -> Math.sin(
  // driverXbox.getRawAxis(
  // 4) *
  // Math.PI)
  // *
  // (Math.PI *
  // 2),
  // () -> Math.cos(
  // driverXbox.getRawAxis(
  // 2) *
  // Math.PI)
  // *
  // (Math.PI *
  // 2))
  // .headingWhile(true)
  // .translationHeadingOffset(true)
  // .translationHeadingOffset(Rotation2d.fromDegrees(
  // 0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // NamedCommands.registerCommand("ClimberUp", new ClimberUp(climber));
    // NamedCommands.registerCommand("ClimberDown", new ClimberDown(climber));
    // Configure the trigger bindings
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    initAutomonousChooser();
    drivebase.zeroGyro();
    NamedCommands.registerCommand("Test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("shootTillEmpty",
        new ShootAtHubCommand(shooter, turret, AutonConstants.MAX_SHOOTER_DOWNTIME));
    NamedCommands.registerCommand("shootForever", new ShootAtHubCommand(shooter, turret, null));
    NamedCommands.registerCommand("deployIntake", Commands.runOnce(() -> intake.deployIntake(), intake));
    NamedCommands.registerCommand("retractIntake", Commands.runOnce(() -> intake.retractIntake(), intake));
    NamedCommands.registerCommand("runIntake", Commands.runOnce(() -> intake.runIntake(), intake));
    NamedCommands.registerCommand("wiggleIntake", new WiggleIntakeCommand(intake));
  }

  private void initAutomonousChooser() {

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    // Command driveFieldOrientedDirectAngleKeyboard =
    // drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // Command driveFieldOrientedAnglularVelocityKeyboard =
    // drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard =
    // drivebase.driveWithSetpointGeneratorFieldRelative(
    // driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      // Sim Controls
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      // driveDirectAngleKeyboard.driveToPose(() -> target,
      // new ProfiledPIDController(5,
      // 0,
      // 0,
      // new Constraints(5, 2)),
      // new ProfiledPIDController(5,
      // 0,
      // 0,
      // new Constraints(Units.degreesToRadians(360),
      // Units.degreesToRadians(180))));
      // driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new
      // Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.button(2).whileTrue(Commands.runEnd(() ->
      // driveDirectAngleKeyboard.driveToPoseEnabled(true),
      // () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest()) {
      // Test Controls
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      // Teleop Controls

      //driverXbox.a().onTrue(());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.a().whileTrue(drivebase.rotateToAngle((((int)drivebase.getHeading().getDegrees()) / 90) * 90 + 45));
      // TODO Make a drive over the bump command and bind it to the B button
      // driverXbox.b().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // TODO Make a drive to a known pose command and bind it to the Y button
      // driverXbox.y().whileTrue(drivebase.driveToPoseCommand(new Pose2d(new
      // Translation2d(1, 4), Rotation2d.fromDegrees(90))));
      // driverXbox.povRight().whileTrue(Commands.runOnce(climber::moveUp,
      // climber).repeatedly());
      // driverXbox.povLeft().whileTrue(Commands.runOnce(climber::moveDown,
      // climber).repeatedly());
      // driverXbox.povUp().whileTrue(Commands.runOnce(climber::climberUp,
      // climber).repeatedly());
      // driverXbox.povDown().whileTrue(Commands.runOnce(climber::climberDown,
      // climber).repeatedly());
      // driverXbox.leftBumper().onTrue(Commands.runOnce(climber::resetHeight,
      // climber));

      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
      driverXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

      operatorXbox.leftBumper().onTrue(Commands.runOnce(shooter::toggleAdaptiveShooting, shooter));
      operatorXbox.leftBumper().onTrue(Commands.runOnce(turret::activateAdaptiveMode, turret));
      operatorXbox.leftBumper().onTrue(
          Commands.runOnce(
              () -> shooter.setTarget(FieldMeasurements.getPassingPosition(drivebase.getPose().getTranslation())),
              shooter));
      operatorXbox.leftBumper().onTrue(
          Commands.runOnce(
              () -> turret.setTarget(FieldMeasurements.getPassingPosition(drivebase.getPose().getTranslation())),
              turret));

      operatorXbox.rightBumper().onTrue(Commands.runOnce(shooter::toggleAdaptiveShooting, shooter));
      operatorXbox.rightBumper().onTrue(Commands.runOnce(turret::activateAdaptiveMode, turret));
      operatorXbox.rightBumper().onTrue(
          Commands.runOnce(() -> shooter.setTarget(FieldMeasurements.getHubPosition()), shooter));
      operatorXbox.rightBumper().onTrue(
          Commands.runOnce(() -> turret.setTarget(FieldMeasurements.getHubPosition()), turret));

      // operatorXbox.povLeft().whileTrue(Commands.runOnce(turret::turnLeft,
      // turret).repeatedly());
      // operatorXbox.povRight().whileTrue(Commands.runOnce(turret::turnRight,
      // turret).repeatedly());

      operatorXbox.b().whileTrue(Commands.runOnce(intake::runIntake, intake).repeatedly());
      operatorXbox.x().whileTrue(Commands.runOnce(intake::spitIntake, intake).repeatedly());
      operatorXbox.a().onTrue(Commands.runOnce(intake::toggleDeployment, intake));
      operatorXbox.y().onTrue(Commands.runOnce(turret::deactivateAdaptiveMode, turret));
      operatorXbox.povUp().whileTrue(Commands.runOnce(intake::moveIn, intake).repeatedly());
      operatorXbox.povDown().whileTrue(Commands.runOnce(intake::moveOut, intake).repeatedly());
      operatorXbox.povLeft().onTrue(Commands.runOnce(shooter::nudgeWeaker, shooter));
      operatorXbox.povRight().onTrue(Commands.runOnce(shooter::nudgeStronger, shooter));
      turret.setDefaultCommand(
          Commands.run(
              () -> turret.changeOffset(
                  MathUtil.applyDeadband(operatorXbox.getRightX(), OperatorConstants.DEADBAND)
                      * TurretConstants.TURRET_SPEED),
              turret));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand(autoChooser.getSelected().getName());
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  // public void climberUp() {
  // climber.climberUp();
  // }

  public void setLEDPattern(LEDPattern pattern) {
    leds.setPatternCommand(pattern);
  }

  public void setTarget(Translation2d target) {
    turret.setTarget(target);
    shooter.setTarget(target);
  }

  public void stopShooting() {
    shooter.stop();
  }
}