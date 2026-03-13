// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.util.FieldMeasurements;

public final class ShootAtHubCommand extends Command{
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  // return Commands.sequence(subsystem.exampleMethodCommand(), new
  // ExampleCommand(subsystem));
  // }

  private final Shooter shooter;
  private final Turret turret;

  public ShootAtHubCommand(Shooter shooter, Turret turret) {
    this.shooter = shooter;
    this.turret = turret;
    addRequirements(shooter, turret);
  }

  /**
   * The initial subroutine of a command. Called once when the command is
   * initially scheduled.
   */
  @Override
  public void initialize() {
    shooter.setTarget(FieldMeasurements.getHubPosition());
    turret.setTarget(FieldMeasurements.getHubPosition());
    shooter.toggleAdaptiveShooting();
    turret.activateAdaptiveMode();
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled.
   * (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes --
   * indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p>
   * <p>
   * Returning false will result in the command never ending automatically. It may
   * still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return
   * true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such
   * an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return shooter.timeSinceLastShot() > AutonConstants.MAX_SHOOTER_DOWNTIME;
  }

  /**
   * The action to take when the command ends. Called when either the command
   * finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    shooter.toggleAdaptiveShooting();
    turret.activateAdaptiveMode();
  }
}