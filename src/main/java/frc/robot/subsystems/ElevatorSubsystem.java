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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  public double k_ElevatorP = ElevatorConstants.kP;
  public double k_ElevatorI = ElevatorConstants.kI;
  public double k_ElevatorD = ElevatorConstants.kD;
  
  public ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

  public ElevatorSubsystem() {
    m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);

    SparkMaxConfig c_base = new SparkMaxConfig();

    // This config will be applied to both motors
    c_base
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kCurrentLimit);
    c_base.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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

    m_timer = new Timer();
    m_timer.start();
  }

  public boolean atTargetPosition() {
    return Math.abs(((e_shepherd.getPosition())) - m_setpoint) < ElevatorConstants.kPositionTolerance;
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

    

  public void setDriveStates(TrapezoidProfile.State start, TrapezoidProfile.State end) {
    m_shepherd.setSetpoint(
        ExampleSmartMotorController.PIDMode.kPosition,
        start.position,
        m_feedforward.calculateWithVelocities(start.velocity, end.velocity) / RobotController.getBatteryVoltage());
  }

  public Command profiledDriveDistance(double distance) {
    return startRun(
            () -> {
              // Restart timer so profile setpoints start at the beginning
              m_timer.restart();
            },
            () -> {
              // Current state never changes, so we need to use a timer to get the setpoints we need
              // to be at
              var currentTime = m_timer.get();
              var currentSetpoint =
                  m_profile.calculate(currentTime, new State(), new State(distance, 0));
              var nextSetpoint =
                  m_profile.calculate(
                      currentTime + 2, new State(), new State(distance, 0));
              setDriveStates(currentSetpoint, currentSetpoint, nextSetpoint, nextSetpoint);
            })
        .until(() -> m_profile.isFinished(0));
  }

  public void runAutomatic() {
    if (m_profile != null) {
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

    SmartDashboard.putBoolean("Motors ?=@ Setpoint", atTargetPosition());
    
    /*
    double m_ElevatorP = SmartDashboard.getNumber("P", k_ElevatorP);
    if(m_ElevatorP != k_ElevatorP) {k_ElevatorP = m_ElevatorP; }
    double m_ElevatorI = SmartDashboard.getNumber("I", k_ElevatorI);
    if(m_ElevatorI != k_ElevatorI) {k_ElevatorI = m_ElevatorI; }
    double m_ElevatorD = SmartDashboard.getNumber("D", k_ElevatorD);
    if(m_ElevatorD != k_ElevatorD) {k_ElevatorD = m_ElevatorD; }

    SparkMaxConfig c_pid = new SparkMaxConfig();
    c_pid.closedLoop.pid(k_ElevatorP, k_ElevatorI, k_ElevatorD);
    m_shepherd.configure(c_pid, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_pid, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    */
  }
}
