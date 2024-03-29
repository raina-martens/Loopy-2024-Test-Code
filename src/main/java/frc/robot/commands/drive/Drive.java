package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class Drive extends DriveCommandBase {

  private final DriveSubsystem driveSubsystem;

  private final DoubleSupplier leftJoystickY, leftJoystickX, rightJoystickX;
  private final BooleanSupplier isFieldRelative;

  /**
   * The command for driving the robot using joystick inputs.
   * @param driveSubsystem The subsystem for the swerve drive
   * @param visionSubsystem The subsystem for vision measurements
   * @param leftJoystickY The joystick input for driving forward and backwards
   * @param leftJoystickX The joystick input for driving left and right
   * @param rightJoystickX The joystick input for turning
   * @param isFieldRelative The boolean supplier if the robot should drive
   * field relative
   */
  public Drive(DriveSubsystem driveSubsystem, DoubleSupplier leftJoystickY, DoubleSupplier leftJoystickX, DoubleSupplier rightJoystickX, BooleanSupplier isFieldRelative) {
    super(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    this.leftJoystickY = leftJoystickY;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.isFieldRelative = isFieldRelative;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Drives the robot
    driveSubsystem.drive(
      leftJoystickY.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
      leftJoystickX.getAsDouble() * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
      rightJoystickX.getAsDouble() * DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
      isFieldRelative.getAsBoolean()
    );

    // Runs all the code from DriveCommandBase that estimates pose
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    // When the command ends, it stops the robot
    driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
