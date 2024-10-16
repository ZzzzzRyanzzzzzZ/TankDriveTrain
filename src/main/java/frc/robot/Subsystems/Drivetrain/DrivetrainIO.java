package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public interface DrivetrainIO 
{
    @AutoLog
    public static class DrivetrainIOInputs {
        public double leftOutputVolts = 0.0;
        public double rightOutputVolts = 0.0;

        public double leftVelocityMetersPerSecond = 0.0;
        public double rightVelocityMetersPerSecond = 0.0;

        public double leftPositionMeters = 0.0;
        public double rightPositionMeters = 0.0;

        public double[] leftCurrentAmps = new double[0];
        public double[] leftTempCelsius = new double[0];
        public double[] rightCurrentAmps = new double[0];
        public double[] rightTempCelsius = new double[0];
    }

    public abstract void updateInputs(DrivetrainIOInputs inputs);

    public abstract void setVolts(double left, double right);

    
}