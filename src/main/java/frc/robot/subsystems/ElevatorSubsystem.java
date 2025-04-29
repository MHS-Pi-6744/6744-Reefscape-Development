package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Simulation;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax m_shepherd;
  private SparkMax m_sheep;

  private SparkClosedLoopController p_shepherd;
  private SparkClosedLoopController p_sheep;

  private SparkAbsoluteEncoder e_cal;

  private RelativeEncoder e_shepherd;
  private RelativeEncoder e_sheep;

  private SparkMaxConfig c_shepherd;
  private SparkMaxConfig c_sheep;

  private double m_setpoint;

  private final ElevatorSim m_simulation;
  private final DCMotor m_shepherdType;
  private final SparkMaxSim s_shepherd;

  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              10,
              90));

  public ElevatorSubsystem() {
    m_shepherd = new SparkMax(ElevatorConstants.kShepherdCanId, SparkMax.MotorType.kBrushless);
    m_sheep = new SparkMax(ElevatorConstants.kSheepCanId, SparkMax.MotorType.kBrushless);

    c_shepherd = Configs.ElevatorSubsystem.shepherdConfig;
    c_sheep = Configs.ElevatorSubsystem.sheepConfig;

    m_shepherd.configure(c_shepherd, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_sheep.configure(c_sheep, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_setpoint = ElevatorConstants.kStartingPosition;

    p_shepherd = m_shepherd.getClosedLoopController();
    p_sheep = m_sheep.getClosedLoopController();

    e_shepherd = m_shepherd.getEncoder();
    e_sheep = m_sheep.getEncoder();

    e_cal = m_shepherd.getAbsoluteEncoder();

    e_shepherd.setPosition(0);
    e_sheep.setPosition(0);

    // e_shepherd.setPosition(e_cal.getPosition());
    // e_sheep.setPosition(e_cal.getPosition());


    m_shepherdType = DCMotor.getNEO(1);
    s_shepherd = new SparkMaxSim(m_shepherd, m_shepherdType);

    m_simulation = new ElevatorSim(
      m_shepherdType,
      ElevatorConstants.kHolyRatio,
      ElevatorConstants.Simulation.kCarriageMassKg,
      ElevatorConstants.Simulation.kElevatorDrumRadius,
      Units.inchesToMeters(ElevatorConstants.kRevSoftLimit),
      Units.inchesToMeters(1/*ElevatorConstants.kFwdSoftLimit*/),
      false,
      1
    );
  }

  public boolean atTargetPosition() {
    return Math.abs(avgEncoderPos() - m_setpoint) < ElevatorConstants.kPositionTolerance;
  }

  public double avgEncoderPos() {
    return (e_shepherd.getPosition() + e_sheep.getPosition()) / 2;
  }

  public void setTargetPosition(double setpoint) {
    m_setpoint = setpoint;
    moveToSetpoint();
  }

  private void moveToSetpoint() {
    p_shepherd.setReference(m_setpoint, ControlType.kMAXMotionPositionControl);
  }

  public void stickControl(double stick) {
    m_shepherd.set(stick);
  }

  public Command resetElevator() {
    return run(() -> e_shepherd.setPosition(0));
  };

  public Command slowBottom() {
    return startEnd(
      () -> m_shepherd.set(-0.1),
      () -> m_shepherd.set(0)
      );
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
    SmartDashboard.putNumber("Calibrator Position", e_cal.getPosition());
    SmartDashboard.putNumber("Calibrator Velocity", e_cal.getVelocity());
    SmartDashboard.putNumber("Sheep Position", e_sheep.getPosition());
    SmartDashboard.putNumber("Sheep Velocity", e_sheep.getVelocity());
    SmartDashboard.putNumber("Shepherd Position", e_shepherd.getPosition());
    SmartDashboard.putNumber("Shepherd Velocity", e_shepherd.getVelocity());
    SmartDashboard.putNumber("Setpoint", m_setpoint);
    SmartDashboard.putBoolean("At Target", atTargetPosition());
  
    m_elevatorMech2d.setLength(m_setpoint);
  }

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putData("Sim Test Thingy", m_mech2d);
    m_simulation.setInput(m_shepherd.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_simulation.update(0.020);
    e_shepherd.setPosition(Units.metersToInches(m_simulation.getPositionMeters()));
  }
}
