package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax m_shepherd;
  private SparkMax m_sheep;

  private SparkClosedLoopController p_shepherd;
  private SparkClosedLoopController p_sheep;

  private RelativeEncoder e_shepherd;
  private RelativeEncoder e_sheep;

  private SparkMaxConfig c_shepherd;
  private SparkMaxConfig c_sheep;

  private double m_setpoint;
  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;
  private TrapezoidProfile.State m_targetState;
  

  public ElevatorSubsystem() {
    m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);

    SparkMaxConfig c_base = new SparkMaxConfig();

    c_base
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);
    c_base.encoder
      .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor);
    c_base.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
      .outputRange(-1, 1);
    c_base.softLimit
      .forwardSoftLimit(ElevatorConstants.kFwdSoftLimit)
      .reverseSoftLimit(ElevatorConstants.kRevSoftLimit)
      .reverseSoftLimitEnabled(true)
      .forwardSoftLimitEnabled(true);
    
    c_sheep = c_base;
    c_shepherd = c_base;

    c_shepherd
      .inverted(false);
    c_sheep
      .inverted(true);

    m_shepherd.configure(c_base, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_sheep, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_setpoint = ElevatorConstants.kStartingPosition;

    p_shepherd = m_shepherd.getClosedLoopController();
    p_sheep = m_sheep.getClosedLoopController();

    e_shepherd = m_shepherd.getEncoder();
    e_sheep = m_sheep.getEncoder();

    e_shepherd.setPosition(m_setpoint);
    e_sheep.setPosition(m_setpoint);
    
    p_shepherd.setReference(ElevatorConstants.kStartingPosition, ControlType.kPosition);
    p_sheep.setReference(ElevatorConstants.kStartingPosition, ControlType.kPosition);

    m_timer = new Timer();
    m_timer.start();
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
    }

    p_sheep.setReference(
        m_setpoint, SparkMax.ControlType.kPosition);
  
  
    p_shepherd.setReference(
        m_setpoint, SparkMax.ControlType.kPosition);
  }
}
