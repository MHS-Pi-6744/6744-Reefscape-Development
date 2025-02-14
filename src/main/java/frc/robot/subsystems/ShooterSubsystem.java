package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ColorSwitch;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.util.Color;




public class ShooterSubsystem extends SubsystemBase{

    // The shooter motor
    private SparkMax m_shooterMotor = new SparkMax(19, MotorType.kBrushless);
  
    // The shooter encoder (set up but not used yet - we may need it later) 
    private RelativeEncoder m_shootEncoder = m_shooterMotor.getEncoder(); 

    private final ShooterSubsystem m_robotDrive = new ShooterSubsystem();


    private final I2C.Port colorSensorPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(colorSensorPort);
  




    // DriveSubsystem constructor - creates & initializes DriveSubsystem object
    public ShooterSubsystem(){
    
        // Create new SPARK MAX configuration objects. These will store the
        // configuration parameters for the SPARK MAXes that we will set below.
        SparkMaxConfig shootMotorConfig = new SparkMaxConfig();
    
        // Set current limit and idle mode for the shooter motor
        shootMotorConfig
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake);

        // Apply the configuration settings to the shooter motor SPARK MAX   
        // - kResetSafeParameters is used to get the SPARK MAX to a known state. This
        //     is useful in case the SPARK MAX is replaced.
        // - kPersistParameters is used to ensure the configuration is not lost when
        //     the SPARK MAX loses power. This is useful for power cycles that may occur
        //     mid-operation.
     
        m_shooterMotor.configure(shootMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Zero shooter encoder on initialization
        m_shootEncoder.setPosition(0);
    }

    //Shooter Commands

    public Command shooterCommand() {
        return startEnd(
            () -> m_shooterMotor.set(ShooterConstants.k_shooterSpeed), 
            () -> m_shooterMotor.set(0));
            
    } 

    public boolean isWhite(boolean isWhiteValue){
        int blue = m_colorSensor.getBlue();
        int red = m_colorSensor.getRed();
        int green = m_colorSensor.getGreen();

        if (red > 200 && green > 200 && blue > 200){
            isWhiteValue = true;
        } else {
            isWhiteValue = false;
        }

        return isWhiteValue;       
      }
    
    public Command shooterCommand1(){
        return new Command() {
            public void initialize(){
                if (m_robotDrive.isWhite(false)){
                                    m_shooterMotor.set(1);
                                }
                            }
                        };
                    } 
                
    public Command slowShooterCommand() {
        return startEnd(
            () -> m_shooterMotor.set(ShooterConstants.k_slowShooter), 
            () -> m_shooterMotor.set(0));
    }   
    public Command shooterReleaseCommand() {
        return startEnd(
            () -> m_shooterMotor.set(-ShooterConstants.k_shooterSpeed), 
            () -> m_shooterMotor.set(0));
    }    

    // Shoot coral by turning the shootor wheels a distance in inches ///////where is distance set up?////////
    // Let's figure out how to implement this later, when we know we need it. I think we have to use closed loop controller,
    // and we may have to change all shooter commands to distance commands with various speeds.
    // 
    //  public Command shootDistCommand(double distance) {
    //    return startEnd(
    //        () -> m_shooterMotor.set(ShooterConstants.k_shootDistance), 
    //        () -> m_shooterMotor.set(0));
    //  }


    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    m_shooterMotor.getOutputCurrent();

    SmartDashboard.putNumber("Shooter Motor Output", m_shooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Motor Current", m_shooterMotor.getOutputCurrent());

    SmartDashboard.putNumber("Shooter Motor P", m_shootEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Motor V", m_shootEncoder.getVelocity());
    }
}