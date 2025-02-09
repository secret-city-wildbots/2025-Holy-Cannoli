package frc.robot.Utility;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;

import frc.robot.Utility.ExtraMotor.MotorBrand;

public class FollowerMotors {
    public int id;
    public TalonFX leaderTFX;
    public TalonFX[] followerTFX;
    public SparkMax leaderSPM;
    public SparkMax[] followerSPM;
    public TalonFXConfiguration followerConfigTFX = new TalonFXConfiguration();

    /**
     * Used for a basic group of follower motors to follow a leader motor
     * the leader motor and all followers can be accessed through .leaderTFX and .followerTFX respectively for TFX,
     * and .leaderSPM and .followerSPM for Spark max
     * note: while it supports both TFX and SPM, you cannot mixmatch (ex: spark max leader and talonFX follower: BAD)
     * @param leaderID id of leader motor
     * @param followerIDs ids of followers
     * @param type type of motor, TalonFX or SparkMax
     */
    public FollowerMotors(int leaderID, int[] followerIDs, MotorBrand type) {
        if (type == MotorBrand.TFX) { //currently only works for TFX bc im llazy

            followerConfigTFX.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            leaderTFX = new TalonFX(leaderID);
            leaderTFX.getDutyCycle().setUpdateFrequency(50);
            leaderTFX.getMotorVoltage().setUpdateFrequency(50);
            leaderTFX.getTorqueCurrent().setUpdateFrequency(50);

            for (int motor = 0; motor < followerIDs.length; motor++) {
                followerTFX[motor] = new TalonFX(followerIDs[motor]);
                followerTFX[motor].setControl(new Follower(leaderID, false));
                followerTFX[motor].getConfigurator().apply(followerConfigTFX);
            }
        } else {
            throw new Error("Currently doesn't support SPM bc why would you ever group neos? Just use a kraken");
        }
    }
}
