package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class Setpoint extends Command { 
    
  private final ElevatorSubsystem m_elevator;
  private final double m_position;

    
  public Setpoint(double position, ElevatorSubsystem elevator) {
    m_position = position;
    m_elevator = elevator;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevator.setTargetPosition(m_position);
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_elevator.atTargetPosition();
  }
}
