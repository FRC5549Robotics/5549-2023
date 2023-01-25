package frc.lib;

public interface DriveController {
    Object getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getStateDistance();
}