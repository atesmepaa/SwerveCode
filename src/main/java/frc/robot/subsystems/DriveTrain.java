package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveContants;

public class DriveTrain extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveModule frontLeft  = new SwerveModule(DriveContants.flDriveCanID, DriveContants.flAngleCanID, DriveContants.flChassisAngularOffset, "fl", 0.01958944275,  3.3008210659);
    private final SwerveModule frontRight = new SwerveModule(DriveContants.frDriveCanID, DriveContants.frAngleCanID, DriveContants.frChassisAngularOffset, "fr", 0.01469208206,  3.29592370986);
    private final SwerveModule backLeft   = new SwerveModule(DriveContants.blDriveCanID, DriveContants.blAngleCanID, DriveContants.blChassisAngularOffset, "bl", 0.01958944275,  3.3008210659);
    private final SwerveModule backRight  = new SwerveModule(DriveContants.brDriveCanID, DriveContants.brAngleCanID, DriveContants.brChassisAngularOffset, "br", 0.01469208206,  3.29102635383);

    // ── Control / Estimation ──────────────────────────────────────────────────
    private final PIDController turnPID = new PIDController(0.01, 0, 0);
    private final SwerveDrivePoseEstimator poseEstimator;

    // ── Telemetry ─────────────────────────────────────────────────────────────
    private final Field2d m_field = new Field2d();

    private final StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("RobotPoseStruct", Pose2d.struct)
            .publish();

    private final StructArrayPublisher<SwerveModuleState> statePublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
            .publish();

    // ─────────────────────────────────────────────────────────────────────────
    public DriveTrain() {
        SmartDashboard.putData("Field", m_field);

        zeroHeading();

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveContants.kDriveKinematics,
            getRotation2d(),
            getModulePositions(),
            new Pose2d()
        );
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999));

        turnPID.enableContinuousInput(-180, 180);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());
        posePublisher.set(getPose());
        m_field.setRobotPose(getPose());

        frontLeft.updateCalibration();
        frontRight.updateCalibration();
        backLeft.updateCalibration();
        backRight.updateCalibration();

        statePublisher.set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });

        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    // ── Drive methods ─────────────────────────────────────────────────────────

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed    * DriveContants.maxSpeedMetersPerSecond,
                ySpeed    * DriveContants.maxSpeedMetersPerSecond,
                rotation  * DriveContants.maxAngularSpeed,
                getRotation2d())
            : new ChassisSpeeds(
                xSpeed   * DriveContants.maxSpeedMetersPerSecond,
                ySpeed   * DriveContants.maxSpeedMetersPerSecond,
                rotation * DriveContants.maxAngularSpeed);

        desaturateAndSet(DriveContants.kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void setX() {
        frontLeft.setDesiredState( new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(  new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState( new SwerveModuleState(0, Rotation2d.fromDegrees( 45)));
    }

    // ── Module helpers ────────────────────────────────────────────────────────

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        desaturateAndSet(desiredStates);
    }

    private void desaturateAndSet(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveContants.maxSpeedMetersPerSecond);
        frontLeft.setDesiredState( states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(  states[2]);
        backRight.setDesiredState( states[3]);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    // ── Pose / Odometry ───────────────────────────────────────────────────────

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    // ── Gyro ──────────────────────────────────────────────────────────────────

    public Rotation2d getRotation2d() {
        double angle = navx.getAngle();
        return Rotation2d.fromDegrees(DriveContants.gyroReversed ? angle : -angle);
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public void zeroHeading() {
        navx.reset();
    }

    // ── Misc ──────────────────────────────────────────────────────────────────

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveContants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(),
            backLeft.getState(),  backRight.getState()
        );
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
}