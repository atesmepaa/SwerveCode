package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class DriveContants {
        
        public static final int flDriveCanID = 3;
        public static final int flAngleCanID = 4;
        
        public static final int frDriveCanID = 1;
        public static final int frAngleCanID = 2;
        
        public static final int blDriveCanID = 5;
        public static final int blAngleCanID = 6;
        
        public static final int brDriveCanID = 7;
        public static final int brAngleCanID = 8;

        public static final double flChassisAngularOffset = Math.toRadians(274);
        public static final double frChassisAngularOffset = Math.toRadians(250);
        public static final double blChassisAngularOffset = Math.toRadians(65);
        public static final double brChassisAngularOffset = Math.toRadians(72);

        // Senin kinematiğin direkt bu değerleri aldığı için / 2.0 olarak ayarlandı.
        public static final double kTrackWidth = Units.inchesToMeters(26.06) / 2.0;
        public static final double kWheelBase = Units.inchesToMeters(21.34) / 2.0;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase, kTrackWidth),   // Front Left
                new Translation2d(kWheelBase, -kTrackWidth),  // Front Right
                new Translation2d(-kWheelBase, kTrackWidth),  // Back Left
                new Translation2d(-kWheelBase, -kTrackWidth)  // Back Right
        );

        public static final double maxSpeedMetersPerSecond = 10.0;
        public static final double maxAngularSpeed = 10 * Math.PI;

        public static final boolean gyroReversed = true;
    }

    public static final class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(3.0); 
        
        public static final double drivingMotorReduction = 5.14; 
        
        public static final double driveWheelFreeSpeedRps = (5676.0 / 60.0) / drivingMotorReduction;

        public static final int drivingMotorCurrentLimit = 40;
        public static final int turningMotorCurrentLimit = 20;
    }

    public static final class SwerveConfigs {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig angleConfig = new SparkMaxConfig();

        static {
            double driveFactor = ModuleConstants.wheelDiameterMeters * Math.PI / ModuleConstants.drivingMotorReduction;
            double driveVelocityFeedForward = 1.0 / ModuleConstants.driveWheelFreeSpeedRps;

            driveConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.drivingMotorCurrentLimit)
                    .openLoopRampRate(0.25)
                    .closedLoopRampRate(0.35);

            driveConfig.encoder
                    .positionConversionFactor(driveFactor) // Metre
                    .velocityConversionFactor(driveFactor / 60.0); // Metre/Saniye

            driveConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.055, 0, 0.01)
                    .velocityFF(driveVelocityFeedForward)
                    .outputRange(-1, 1);

            double angleFactor = (2 * Math.PI) / 3.4;

            angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.turningMotorCurrentLimit)
                    .closedLoopRampRate(0.1);

            angleConfig.analogSensor
                    .positionConversionFactor(angleFactor)
                    .velocityConversionFactor(angleFactor / 60.0);

            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAnalogSensor)
                    .pid(0.58, 0, 0.0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, angleFactor + 0.5);
        }
    }
}