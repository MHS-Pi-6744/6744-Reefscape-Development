package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
    public static final class ElevatorSubsystem {
      public static final SparkMaxConfig shepherdConfig = new SparkMaxConfig();
      public static final SparkMaxConfig sheepConfig = new SparkMaxConfig();
      
      static {
      shepherdConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
        .inverted(false);
      shepherdConfig.absoluteEncoder
        .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
        .inverted(true);
      shepherdConfig.encoder
        .positionConversionFactor(24)
        .velocityConversionFactor(24);
      shepherdConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .outputRange(-1, 1)
        .maxMotion    
        .maxVelocity(800)
        .maxAcceleration(6000)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(ElevatorConstants.kPositionTolerance);
      shepherdConfig.softLimit
        .forwardSoftLimit(ElevatorConstants.kFwdSoftLimit)
        .reverseSoftLimit(ElevatorConstants.kRevSoftLimit)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimitEnabled(true);
      
      sheepConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
        .inverted(true)
        .follow(ElevatorConstants.kShepherdCanId, true);
      sheepConfig.absoluteEncoder
        .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
        .inverted(true);
      sheepConfig.encoder
        .positionConversionFactor(8.36);
      sheepConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .outputRange(-1, 1)
        .maxMotion    
        .maxVelocity(800)
        .maxAcceleration(6000)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(ElevatorConstants.kPositionTolerance);
      sheepConfig.softLimit
        .forwardSoftLimit(ElevatorConstants.kFwdSoftLimit)
        .reverseSoftLimit(ElevatorConstants.kRevSoftLimit)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimitEnabled(true);
      }
    }
}
