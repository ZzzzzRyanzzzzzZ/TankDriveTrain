// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drivetrain;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
/** Add your docs here. */
public class DrivetrainIOSim implements DrivetrainIO 
    {
        public DifferentialDrivetrainSim ddsm = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k8p45, KitbotWheelSize.kEightInch, null);
        public double leftVolts;
        public double rightVolts;


        @Override
        public void updateInputs(DrivetrainIOInputs inputs) 
        {

            physicsSim.update(0.020);
            
            inputs.leftOutputVolts = leftVolts;
            inputs.rightOutputVolts = rightVolts;
            inputs.leftVelocityMetersPerSecond = physicsSim.getLeftVelocityMetersPerSecond();
            inputs.rightVelocityMetersPerSecond = physicsSim.getRightVelocityMetersPerSecond();
            inputs.leftVelocityMetersPerSecond = physicsSim.getLeftVelocityMetersPerSecond();
            inputs.rightVelocityMetersPerSecond = physicsSim.getRightVelocityMetersPerSecond();
    
        }
    DifferentialDrivetrainSim physicsSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDoubleFalcon500PerSide,
    KitbotGearing.k8p45,
    KitbotWheelSize.kSixInch,
    null);
    
    
        @Override
        public void setVolts(double left, double right) 
        {
            ddsm.setInputs(left, right);
        }       

    }
