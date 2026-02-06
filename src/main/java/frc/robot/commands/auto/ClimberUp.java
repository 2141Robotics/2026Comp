package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * <a href=
 * "https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java">...</a>
 */
public class ClimberUp extends Command {

    Climber climber;
    

  public ClimberUp(Climber climber) {
    this.climber = climber;
    
    addRequirements(this.climber);
  }

  /**
   * The initial subroutine of a command. Called once when the command is
   * initially scheduled.
   */
  @Override
  public void initialize() {
    System.out.println("Climber Up");
    climber.climberUp();
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
    return climber.atHeight();
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
    
  }
}