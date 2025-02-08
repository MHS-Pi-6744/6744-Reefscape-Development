package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
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

  private AbsoluteEncoder e_shepherd;
  private AbsoluteEncoder e_sheep;

  private SparkMaxConfig c_shepherd;
  private SparkMaxConfig c_sheep;

  private double m_setpoint;

  public double k_ElevatorP = ElevatorConstants.kP;
  public double k_ElevatorI = ElevatorConstants.kI;
  public double k_ElevatorD = ElevatorConstants.kD;
  

  public ElevatorSubsystem() {
    m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);

    SparkMaxConfig c_base = new SparkMaxConfig();

    // This config will be applied to both motors
    c_base
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);
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

    e_shepherd = m_shepherd.getAbsoluteEncoder();
    e_sheep = m_sheep.getAbsoluteEncoder();
    
    p_shepherd.setReference(ElevatorConstants.kStartingPosition, ControlType.kPosition);
    p_sheep.setReference(ElevatorConstants.kStartingPosition, ControlType.kPosition);
  }

  public boolean atTargetPosition() {
    return Math.abs(e_shepherd.getPosition() - m_setpoint) < ElevatorConstants.kPositionTolerance;
  }

  public Command setTargetPosition(double setpoint) {
    return this.runOnce( () -> m_setpoint = setpoint);
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
    SmartDashboard.putNumber("Sheep Position", e_sheep.getPosition());
    SmartDashboard.putNumber("Shepherd Position", e_shepherd.getPosition());
    SmartDashboard.putNumber("Sheep Velocity", e_sheep.getVelocity());
    SmartDashboard.putNumber("Shepherd Velocity", e_shepherd.getVelocity());

    moveToSetpoint();
    SmartDashboard.putNumber("SetPoint", m_setpoint);
    SmartDashboard.putNumber("P", k_ElevatorP);
    SmartDashboard.putNumber("I", k_ElevatorI);
    SmartDashboard.putNumber("D", k_ElevatorD);
   
    double m_ElevatorP = SmartDashboard.getNumber("P", ElevatorConstants.kP);
    if(m_ElevatorP != k_ElevatorP) {k_ElevatorP = m_ElevatorP; }
    double m_ElevatorI = SmartDashboard.getNumber("I", ElevatorConstants.kI);
    if(m_ElevatorI != k_ElevatorI) {k_ElevatorI = m_ElevatorI; }
    double m_ElevatorD = SmartDashboard.getNumber("D", ElevatorConstants.kD);
    if(m_ElevatorD != k_ElevatorD) {k_ElevatorD = m_ElevatorD; }

    SparkMaxConfig c_pid = new SparkMaxConfig();
    c_pid.closedLoop.pid(k_ElevatorP, k_ElevatorI, k_ElevatorD);
    m_shepherd.configure(c_pid, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_pid, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
  }
}
