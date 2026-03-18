package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveConfigs {
    /**
     * This method is for Leader SparkMax Config
     * @param inverted Boolean, should it be inverted?
     * @return a config for SparkMaxes
    */
    public static SparkMaxConfig getConfig(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        // config.smartCurrentLimit(40);
        return config;
    }

    /**
     * This method is for Leader SparkMax Config
     * @param inverted Boolean, should it be inverted?
     * @param leaderID Integer, what sparkmax should it follow
     * @return a config for SparkMaxes
    */
    public static SparkMaxConfig getConfig(boolean inverted, int leaderID) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        // config.smartCurrentLimit(40);
        config.follow(leaderID);
        return config;
    }
}
