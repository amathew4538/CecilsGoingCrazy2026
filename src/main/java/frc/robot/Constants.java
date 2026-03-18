// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Pneumatics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public final static SparkMax m_leftLeader = new SparkMax(4, MotorType.kBrushless); //C: defines sparkmax 4 as front left, as a leader to let us know it's meant to receive input
    public final static SparkMax m_leftFollower = new SparkMax(3, MotorType.kBrushless); //C: defines sparkmax 3 as back left, as a follower to let us know it matches its leaders' input
    public final static SparkMax m_rightLeader = new SparkMax(2, MotorType.kBrushless); //C: ditto of line 11 but for the right and uses sparkmax 2
    public final static SparkMax m_rightFollower = new SparkMax(1, MotorType.kBrushless); //C: ditto of line 12 but for the right and uses sparkmax 1

    public static Gyroscope m_gyroscope;
    public static Pneumatics m_pneumatics;

    public final static PIDController m_pid = new PIDController(0.03, 0.01, 0);

    public final static RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
    public final static RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();


    public final static edu.wpi.first.wpilibj.smartdashboard.Field2d m_Field2d = new Field2d();

    // public final double positionConversion = (Units.inchesToMeters(6) * Math.PI) / 7.29;

    public final static double highGearThreshold = 1.75;   // ! Meters per second (Tune these!)
    public final static double lowGearThreshold = 1.0;

    public final static double lowGearRatio = 20.523724;
    public final static double highGearRatio = 9.261941;

    public static boolean isAutoShiftEnabled = true;

    public static int autoShiftTimer;
  
    public final static LTVUnicycleController controller = new LTVUnicycleController(0.02);

    public final static DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25));

    public static double m_prevLeftDist = 0;
    public static double m_prevRightDist = 0;
    public static double m_totalLeftDist = 0;
    public static double m_totalRightDist = 0;

    public final static SimpleMotorFeedforward m_lowGearFF = new SimpleMotorFeedforward(0.094402, 0.12423, 0.026461);
    public final static SimpleMotorFeedforward m_highGearFF = new SimpleMotorFeedforward(0.05, 0.08, 0.015); // ! Change these

    public static boolean currentlyHighLogger = false;

    public static double wheelCoefficient = 1.0933;
  }
}
