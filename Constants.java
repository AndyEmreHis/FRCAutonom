package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    static class DriveTrainConstants {
        static final double kMaxSpeed = 3.0; // meters per second
        static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        static final double kTrackWidth = 0.33 * 2; // meters
        static final double kWheelRadius = 0.1524; // meters
        static final int kEncoderResolution = 4096;
        static final double distancePerPulse = 2 * Math.PI * kWheelRadius / (double) kEncoderResolution;
    }

    static class FieldConstants {
        static final double length = Units.feetToMeters(47);
        static final double width = Units.feetToMeters(27);
    }

    static class VisionConstants {
        Transform3d robotToCam = new Transform3d(new Translation3d(0.18, 0.135, 0.10), new Rotation3d(0, 0, 0));
        static final String cameraName = "C922_Pro_Stream_Webcam";
    }
}
