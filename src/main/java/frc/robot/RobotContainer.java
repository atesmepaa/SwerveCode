package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {

    private final DriveTrain m_driveTrain = new DriveTrain();
    private final QuestNavSubsystem m_questNav = new QuestNavSubsystem(m_driveTrain);
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureButtonBindings();

        m_driveTrain.setDefaultCommand(
            new RunCommand(
                () -> m_driveTrain.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), 0.1),
                    true
                ),
                m_driveTrain
            )
        );
    }

    private void configureButtonBindings() {

        m_driverController.x().whileTrue(
            new RunCommand(m_driveTrain::setX, m_driveTrain)
        );

        m_driverController.b().onTrue(
            new InstantCommand(
                () -> m_questNav.resetPose(new Pose2d(3.5, 4.0, new Rotation2d(0))),
                m_driveTrain, m_questNav
            )
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}