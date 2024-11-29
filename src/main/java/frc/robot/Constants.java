package frc.robot;

public class Constants {
  public static class VisionConstants {
    public static final String CAMERA_NAME = "camera1";

    public static final double CAMERA_TO_ROBOT_X = 0;
    public static final double CAMERA_TO_ROBOT_Y = 0;
    public static final double CAMERA_TO_ROBOT_Z = 0;
  }

  public static class DrivetrainConstants {
    public static final int MAX_MOTOR_SPEED = 1;

    public static final int LEFT_FRONT_ID = 3;
    public static final int LEFT_BACK_ID = 4;
    public static final int RIGHT_FRONT_ID = 1;
    public static final int RIGHT_BACK_ID = 2;
  }
  
  public static class ArmConstants {
    public static final int MAX_MOTOR_SPEED = 1;

    public static final int LEFT_ID = 5;
    public static final int RIGHT_ID = 6;

    public static class PIDController {
      public static final double PROPOTIONAL = 0;
      public static final double INTEGRAL = 0;
      public static final double DERIVATIVE = 0;
    }
  }

  public static class DriverConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final int LEFT_AXIS = 1; // SIM GUI W/S
    public static final int RIGHT_AXIS = 5; // SIM GUI I/K
  }
}