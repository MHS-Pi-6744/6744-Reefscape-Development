package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax m_shepherd;
  private SparkMax m_sheep;

  private SparkClosedLoopController p_shepherd;
  private SparkClosedLoopController p_sheep;

  private SparkAbsoluteEncoder e_shepherd;
  private SparkAbsoluteEncoder e_sheep;

  private RelativeEncoder r_shepherd;
  private RelativeEncoder r_sheep;

  private SparkMaxConfig c_shepherd;
  private SparkMaxConfig c_sheep;

  private double m_setpoint;

  public double k_ElevatorP = ElevatorConstants.kP;
  public double k_ElevatorI = ElevatorConstants.kI;
  public double k_ElevatorD = ElevatorConstants.kD;
  public double k_ElevatorF = ElevatorConstants.kF;
  

  public ElevatorSubsystem() {
    m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);

    SparkMaxConfig c_base = new SparkMaxConfig();

    // This config will be applied to both motors
    c_base
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);
    c_base.absoluteEncoder
      .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
      .inverted(true);
    c_base.encoder
      .positionConversionFactor(24)
      .velocityConversionFactor(24);
    c_base.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(k_ElevatorP, k_ElevatorI, k_ElevatorD, k_ElevatorF)
      .outputRange(-1, 1)
      .maxMotion    
      .maxVelocity(800)
      .maxAcceleration(6000)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
      .allowedClosedLoopError(ElevatorConstants.kPositionTolerance);
    c_base.softLimit
      .forwardSoftLimit(ElevatorConstants.kFwdSoftLimit)
      .reverseSoftLimit(ElevatorConstants.kRevSoftLimit)
      .reverseSoftLimitEnabled(false)
      .forwardSoftLimitEnabled(false);
    
    c_sheep = c_base;
    c_shepherd = c_base;

    // These configs will be appied to the motors individually
    c_shepherd
      .inverted(false);
    c_sheep
      .inverted(true);

    m_shepherd.configure(c_shepherd, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_sheep, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_setpoint = ElevatorConstants.kStartingPosition;

    p_shepherd = m_shepherd.getClosedLoopController();
    p_sheep = m_sheep.getClosedLoopController();

    r_shepherd = m_shepherd.getEncoder();
    r_sheep = m_sheep.getEncoder();

    e_shepherd = m_shepherd.getAbsoluteEncoder();
    e_sheep = m_sheep.getAbsoluteEncoder();

    r_shepherd.setPosition(e_shepherd.getPosition());
    r_sheep.setPosition(e_sheep.getPosition());
  }

  public boolean atTargetPosition() {
    return Math.abs(e_shepherd.getPosition() - m_setpoint) < ElevatorConstants.kPositionTolerance;
  }

  public void setTargetPosition(double setpoint) {
    m_setpoint = setpoint;
  }

  private void moveToSetpoint() {
    p_shepherd.setReference(m_setpoint, ControlType.kMAXMotionPositionControl);
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
    SmartDashboard.putNumber("Shepherd Position", e_shepherd.getPosition());
    SmartDashboard.putNumber("Shepherd Velocity", e_shepherd.getVelocity());
    SmartDashboard.putNumber("Relative Shepherd Position", r_shepherd.getPosition());
    SmartDashboard.putNumber("Relative Shepherd Velocity", r_shepherd.getVelocity());
    SmartDashboard.putNumber("I Accumulator", p_shepherd.getIAccum());
    SmartDashboard.putNumber("Setpoint", m_setpoint);
    SmartDashboard.putBoolean("At Target", atTargetPosition());
    moveToSetpoint();
  }
}
