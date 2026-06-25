package frc.robot.subsystems;

import java.util.OptionalInt;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav = new QuestNav();
    private final DriveTrain driveTrain;

    private Pose2d lastCorrectedPose = new Pose2d();
    private boolean hasPose = false;
    private boolean questTracking = false;
    private int questBatteryPercent = -1;

    // Robotunuza göre ölçülmüş değerler
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(
            new Translation3d(0.195, 0.285, 0.40),
            new Rotation3d(0.0, 0.0, Math.PI / 2)
        );

    // Düşük std dev = QuestNav'a çok güven
    private static final Matrix<N3, N1> QUEST_STD_DEVS =
        VecBuilder.fill(0.02, 0.02, 0.035);

    public QuestNavSubsystem(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();

        OptionalInt batteryOpt = questNav.getBatteryPercent();
        questBatteryPercent = batteryOpt.isPresent() ? batteryOpt.getAsInt() : -1;

        questTracking = false;

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame frame : frames) {
            if (!frame.isTracking()) continue;

            questTracking = true;

            Pose3d robotPose3d = frame.questPose3d()
                .transformBy(ROBOT_TO_QUEST.inverse());

            Pose2d robotPose2d = robotPose3d.toPose2d();
            lastCorrectedPose = robotPose2d;
            hasPose = true;

            driveTrain.addVisionMeasurement(
                robotPose2d,
                frame.dataTimestamp(),
                QUEST_STD_DEVS
            );
        }

        publishToDashboard();
    }

    private void publishToDashboard() {
        SmartDashboard.putBoolean("Quest/Tracking",  questTracking);
        SmartDashboard.putBoolean("Quest/Connected", questNav.isConnected());
        SmartDashboard.putNumber ("Quest/Battery",   questBatteryPercent);

        if (hasPose) {
            SmartDashboard.putNumber("Quest/X",      lastCorrectedPose.getX());
            SmartDashboard.putNumber("Quest/Y",      lastCorrectedPose.getY());
            SmartDashboard.putNumber("Quest/RotDeg", lastCorrectedPose.getRotation().getDegrees());
        }
    }

    public Pose2d getPose2d() {
        return hasPose ? lastCorrectedPose : new Pose2d();
    }

    public void zeroPose(Pose2d targetRobotPose) {
        Pose3d targetQuestPose = new Pose3d(targetRobotPose)
            .transformBy(ROBOT_TO_QUEST);
        questNav.setPose(targetQuestPose);
        System.out.println("[QuestNav] zeroPose → " + targetRobotPose);
    }

    public void resetPose(Pose2d robotPose) {
        driveTrain.resetOdometry(robotPose);
        zeroPose(robotPose);
        lastCorrectedPose = robotPose;
    }

    public boolean isTracking()       { return questTracking; }
    public int     getBatteryPercent() { return questBatteryPercent; }
    public boolean isConnected()      { return questNav.isConnected(); }
}