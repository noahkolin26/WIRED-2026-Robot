package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Telemetry {

    private static final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("Telemetry");

    private static final HashMap<String, DoublePublisher> doubles = new HashMap<>();
    private static final HashMap<String, BooleanPublisher> booleans = new HashMap<>();
    private static final HashMap<String, StringPublisher> strings = new HashMap<>();
    private static final HashMap<String, IntegerPublisher> integers = new HashMap<>();
    private static final HashMap<String, StructPublisher<Pose2d>> poses = new HashMap<>();
    private static final HashMap<String, Field2d> fields = new HashMap<>();

    public static void putDouble(String key, double value) {

        doubles.computeIfAbsent(key,
            k -> table.getDoubleTopic(k).publish());

        doubles.get(key).set(value);
    }

    public static void putBoolean(String key, boolean value) {

        booleans.computeIfAbsent(key,
            k -> table.getBooleanTopic(k).publish());

        booleans.get(key).set(value);
    }

    public static void putString(String key, String value) {

        strings.computeIfAbsent(key,
            k -> table.getStringTopic(k).publish());

        strings.get(key).set(value);
    }

    public static void putInteger(String key, int value) {

        integers.computeIfAbsent(key,
            k -> table.getIntegerTopic(k).publish());

        integers.get(key).set(value);
    }

    public static void putPose(String key, Pose2d pose) {

        poses.computeIfAbsent(key,
            k -> table.getStructTopic(k, Pose2d.struct).publish());

        poses.get(key).set(pose);
    }

    public static void putFieldPose(String key, Pose2d pose) {

        fields.computeIfAbsent(key, k -> {
            Field2d field = new Field2d();
            SmartDashboard.putData(k, field);
            return field;
        });

        fields.get(key).setRobotPose(pose);
    }

    // this is how we could compare vision estimates, odometry estimates, etc visually (make sure fieldKey = key in putFieldPose)
    public static void putFieldObjectPose(String fieldKey, String objectName, Pose2d pose) {

        fields.computeIfAbsent(fieldKey, k -> {
            Field2d field = new Field2d();
            SmartDashboard.putData(k, field);
            return field;
        });

        fields.get(fieldKey)
            .getObject(objectName)
            .setPose(pose);
    }
}