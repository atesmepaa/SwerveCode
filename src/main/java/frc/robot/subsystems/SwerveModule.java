package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final SparkAnalogSensor angleEncoder;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;

    private final double CAO;
    @SuppressWarnings("unused")
    private final String moduleName;

    private double calibratedMinVolt;
    private double calibratedMaxVolt;

    public SwerveModule(int driveCanID, int angleCanID, double chassisAngularOffset, String name, double minVolt, double maxVolt) {
        this.moduleName = name;
        this.CAO = chassisAngularOffset;
        
        this.calibratedMinVolt = minVolt;
        this.calibratedMaxVolt = maxVolt;

        driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleCanID, MotorType.kBrushed);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getAnalog();

        drivePID = driveMotor.getClosedLoopController();
        anglePID = angleMotor.getClosedLoopController();

        // Konfigürasyonlar harici dosyadan değil, doğrudan Constants altından çağrılıyor
        driveMotor.configure(Constants.SwerveConfigs.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(Constants.SwerveConfigs.angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoders();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRadians(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(CAO));

        // Analog absolute encoder'dan gelen radyan verisine göre optimizasyon
        correctedDesiredState.optimize(Rotation2d.fromRadians(getAngle()));

        drivePID.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        anglePID.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

public double getAngle() {
    double rawVolt = angleEncoder.getVoltage();
    double normalized = (rawVolt - calibratedMinVolt) / (calibratedMaxVolt - calibratedMinVolt);
    normalized = (normalized % 1.0 + 1.0) % 1.0;
    return normalized * 2 * Math.PI;
}

    public void updateCalibration() {
        double currentVolt = angleEncoder.getVoltage(); 

        if (currentVolt < calibratedMinVolt && currentVolt > 0.0001) {
            calibratedMinVolt = currentVolt;
        }

        if (currentVolt > calibratedMaxVolt && currentVolt < 3.3) {
            calibratedMaxVolt = currentVolt;
        }
    }
}