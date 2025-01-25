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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public double k_ElevatorP = ElevatorConstants.kP;
  public double k_ElevatorI = ElevatorConstants.kI;
  public double k_ElevatorD = ElevatorConstants.kD;
  

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
      .pid(k_ElevatorP, k_ElevatorI, k_ElevatorD)
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

  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }
    p_sheep.setReference(
      m_targetState.position, SparkMax.ControlType.kPosition);

    p_shepherd.setReference(
      m_targetState.position, SparkMax.ControlType.kPosition);
  }

  public void setArmCoastMode(){
    SparkMaxConfig c_mod = new SparkMaxConfig();
    c_mod.idleMode(IdleMode.kCoast);
    m_shepherd.configure(c_mod, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_mod, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setArmBrakeMode(){
    SparkMaxConfig c_mod = new SparkMaxConfig();
    c_mod.idleMode(IdleMode.kBrake);
    m_shepherd.configure(c_mod, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_mod, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  @Override
  public void periodic() { // This method will be called once per scheduler run
    SmartDashboard.putNumber("Sheep Position", e_sheep.getPosition());
    SmartDashboard.putNumber("Shepherd Position", e_shepherd.getPosition());
    SmartDashboard.putNumber("Sheep Velocity", e_sheep.getVelocity());
    SmartDashboard.putNumber("Shepherd Velocity", e_shepherd.getVelocity());

    SmartDashboard.putNumber("SetPoint", m_setpoint);
    SmartDashboard.putNumber("P", k_ElevatorP);
    SmartDashboard.putNumber("I", k_ElevatorI);
    SmartDashboard.putNumber("D", k_ElevatorD);
   
    double m_ElevatorP = SmartDashboard.getNumber("P", ElevatorConstants.kP);
    if((m_ElevatorP != k_ElevatorP)) {k_ElevatorP = m_ElevatorP; }
    double m_ElevatorI = SmartDashboard.getNumber("I", ElevatorConstants.kI);
    if((m_ElevatorI != k_ElevatorI)) {k_ElevatorI = m_ElevatorI; }
    double m_ElevatorD = SmartDashboard.getNumber("D", ElevatorConstants.kD);
    if((m_ElevatorD != k_ElevatorD)) {k_ElevatorD = m_ElevatorD; }

    SparkMaxConfig c_pid = new SparkMaxConfig();
    c_pid.closedLoop.pid(k_ElevatorP, k_ElevatorI, k_ElevatorD);
    m_shepherd.configure(c_pid, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_pid, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
  }
}
