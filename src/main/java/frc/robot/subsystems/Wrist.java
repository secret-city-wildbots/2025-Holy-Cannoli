package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Wrist {
  public static final DutyCycleEncoder encoder = new DutyCycleEncoder(1, 1-Units.degreesToRotations(15), -Units.degreesToRotations(15));
}