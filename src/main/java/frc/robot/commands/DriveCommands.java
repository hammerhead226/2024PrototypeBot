package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double NOTE_FORWARD_OFFSET = -0.36;
  private static double sideWaysError = 0;
  private static double wantedSidewaysVelocity = 0;
  private static double wantedRotationVelocity = 0;
  private static double sidewaysAssistEffort = 0;
  private static double rotationAssistEffort = 0;
  private static double forwardConstantVelocity = 0;
  private static PIDController sidewaysPID =
      new PIDController(1.5, 0, 0, Constants.LOOP_PERIOD_SECS);
  private static PIDController rotationPID =
      new PIDController(2.54, 0, 0, Constants.LOOP_PERIOD_SECS);

  private static double counter = 0;

  private DriveCommands() {}

  
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier intakeAssistSupplier,
      BooleanSupplier turnToAmpSupplier) {
    return Commands.run(
        () -> {
          rotationPID.setTolerance(1);
          rotationPID.enableContinuousInput(-180, 180);
          sidewaysPID.setTolerance(0.05460);
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          ChassisSpeeds chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());

          double forwardSpeed = chassisSpeeds.vxMetersPerSecond;

          double sidewaysSpeed = chassisSpeeds.vyMetersPerSecond;

          double rotationSpeed = chassisSpeeds.omegaRadiansPerSecond;

         

          if (turnToAmpSupplier.getAsBoolean()) {
            Rotation2d curreRotation2d = drive.getRotation();
            Rotation2d targeRotation2d;
            // if (DriverStation.getAlliance().get() == Alliance.Blue) {
            //   targeRotation2d = Rotation2d.fromDegrees(-60); //-60 for blue source
            // } else {
            //   targeRotation2d = Rotation2d.fromDegrees(240); //240 for red sjkource
            // }
            targeRotation2d = Rotation2d.fromDegrees(-88); // TODO 90 or -90 for amp, need to test
            rotationPID.setSetpoint(targeRotation2d.getDegrees());

            wantedRotationVelocity =
                Math.toRadians(rotationPID.calculate(curreRotation2d.getDegrees()));

            rotationAssistEffort = wantedRotationVelocity - rotationSpeed * 0.1690;

          } else {
            wantedRotationVelocity = rotationSpeed;
            rotationAssistEffort = 0;
          }

          

          Logger.recordOutput("Wanted Sideways Velocity", wantedSidewaysVelocity);
          Logger.recordOutput("Note Assist Error", sideWaysError);

          Logger.recordOutput("Sideways Assist Effort", sidewaysAssistEffort);
          Logger.recordOutput("Rotation Assist Effort", rotationAssistEffort);

          drive.runVelocity(
              new ChassisSpeeds(
                  MathUtil.clamp(
                      forwardSpeed + forwardConstantVelocity,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec()),
                  MathUtil.clamp(
                      sidewaysSpeed + sidewaysAssistEffort,
                      -drive.getMaxLinearSpeedMetersPerSec(),
                      drive.getMaxLinearSpeedMetersPerSec()),
                  MathUtil.clamp(
                      rotationSpeed + rotationAssistEffort,
                      -drive.getMaxAngularSpeedRadPerSec(),
                      drive.getMaxAngularSpeedRadPerSec())));
        },
        drive);
  }

  

  private static double calculateTime(double velocity, double displacement) {
    double time = displacement / velocity;
    Logger.recordOutput("Time to note", time);

    return time;
  }

  private static double calculateVelocity(double time, double displacement) {
    double velocity = displacement / time;
    Logger.recordOutput("Velocity needed to note", velocity);

    return velocity;
  }
}