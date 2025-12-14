package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ReefscapeOpponentPoses {
    // Map of possible scoring poses and types. For example, Map<"Hoops", Map<"CourtLeft", Pose2d>>
    private static final Map<String, Map<String, Pose2d>> scoringMap = new HashMap<>();
    // Map of possible collecting poses and types. For example, Map<"CollectStation", Map<"StationCenter", Pose2d>>
    private static final Map<String, Map<String, Pose2d>> collectingMap = new HashMap<>();
    // List of starting poses
    private static final List<Pose2d> initialBluePoses = new ArrayList<>();
    private static final List<Pose2d> initialRedPoses = new ArrayList<>();
    // Map of queening poses
    private static final List<Pose2d> queeningPoses = new ArrayList<>();

    public ReefscapeOpponentPoses()
    {
    }

    static
    {
        /// All reef scoring poses
        addScoringPose("Reef", "Reef South Left", PoseConstants.REEF_SOUTH_LEFT_POSE);
        addScoringPose("Reef", "Reef South  Right", PoseConstants.REEF_SOUTH_RIGHT_POSE);
        addScoringPose("Reef", "Reef Southeast Left", PoseConstants.REEF_SOUTHEAST_LEFT_POSE);
        addScoringPose("Reef", "Reef Southeast Right", PoseConstants.REEF_SOUTHEAST_RIGHT_POSE);
        addScoringPose("Reef", "Reef Northeast Left", PoseConstants.REEF_NORTHEAST_LEFT_POSE);
        addScoringPose("Reef", "Reef Northeast Right", PoseConstants.REEF_NORTHEAST_RIGHT_POSE);
        addScoringPose("Reef", "Reef North Left",  PoseConstants.REEF_NORTH_LEFT_POSE);
        addScoringPose("Reef", "Reef North Right",  PoseConstants.REEF_NORTH_RIGHT_POSE);
        addScoringPose("Reef", "Reef Northwest Left",  PoseConstants.REEF_NORTHWEST_LEFT_POSE);
        addScoringPose("Reef", "Reef Northwest Right",  PoseConstants.REEF_NORTHWEST_RIGHT_POSE);
        addScoringPose("Reef", "Reef Southwest Left",  PoseConstants.REEF_SOUTHWEST_LEFT_POSE);
        addScoringPose("Reef", "Reef Southwest Right", PoseConstants.REEF_SOUTHWEST_RIGHT_POSE);
        /// Barge scoring pose
        addScoringPose("Barge", "Net", PoseConstants.BARGE_NET_POSE);
        /// Coral Station Poses
        addCollectingPose("Left Station", "Left", PoseConstants.LEFT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_LEFT));
        addCollectingPose("Left Station", "Center", PoseConstants.LEFT_STATION_POSE);
        addCollectingPose("Left Station", "Right", PoseConstants.LEFT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_RIGHT));
        addCollectingPose("Right Station", "Left", PoseConstants.RIGHT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_LEFT));
        addCollectingPose("Right Station", "Center", PoseConstants.RIGHT_STATION_POSE);
        addCollectingPose("Right Station", "Right", PoseConstants.RIGHT_STATION_POSE.plus(PoseConstants.SLOT_OFFSET_RIGHT));

        queeningPoses.add(new Pose2d(-6, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-5, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-4, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-3, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-2, 0, new Rotation2d()));
        queeningPoses.add(new Pose2d(-1, 0, new Rotation2d()));

        initialBluePoses.add(new Pose2d(15, 6, Rotation2d.fromDegrees(180)));
        initialBluePoses.add(new Pose2d(15, 4, Rotation2d.fromDegrees(180)));
        initialBluePoses.add(new Pose2d(15, 2, Rotation2d.fromDegrees(180)));
        initialRedPoses.add(new Pose2d(1.6, 6, Rotation2d.kZero));
        initialRedPoses.add(new Pose2d(1.6, 4, Rotation2d.kZero));
        initialRedPoses.add(new Pose2d(1.6, 2, Rotation2d.kZero));
    }

    /**
     * Gets a queening pose, removing it from the list.
     *
     * @return a queening pose.
     */
    public static Pose2d getQueeningPose()
    {
        var pose = queeningPoses.get(0);
        queeningPoses.remove(0);
        return pose;
    }

    /**
     * Gets a initial pose, removing it from the list.
     *
     * @return a initial pose.
     */
    public static Pose2d getInitialPose(DriverStation.Alliance alliance)
    {
        Pose2d pose;
        if (alliance == DriverStation.Alliance.Blue) {
            pose = initialBluePoses.get(0);
            initialBluePoses.remove(0);
        } else {
            pose = initialRedPoses.get(0);
            initialRedPoses.remove(0);
        }
        return pose;
    }

    /**
     * Adds a scoring pose.
     *
     * @param poseType The type of pose to add. For example, "Hoops".
     * @param poseName The name of the pose to add. For example, "CourtLeft".
     * @param pose The pose to add.
     * @return this, for chaining.
     */
    public static void addScoringPose(String poseType, String poseName, Pose2d pose) {
        scoringMap.putIfAbsent(poseType, new HashMap<>());
        scoringMap.get(poseType).putIfAbsent(poseName, pose);
        if (scoringMap.get(poseType).get(poseName) != pose) {
            throw new IllegalArgumentException("Failed to add score pose: " + poseName + "/n");
        }
    }

    /**
     * Gets the raw scoring map.
     * Setup as Map<poseType, Map<poseName, Pose2d>>
     *
     * @return the raw scoring map.
     */
    public static Map<String, Map<String, Pose2d>> getRawScoringMap() {
        return scoringMap;
    }

    /**
     * Compiles a compacted scoring map.
     * Setup as Map<poseName, Pose2d>
     *
     * @return a compacted scoring map.
     */
    public static Map<String, Pose2d> getScoringMap() {
        Map<String, Pose2d> scoringPoses = new HashMap<>();
        scoringMap.forEach((poseType, poseMap) ->
                poseMap.forEach((name, pose) -> {
                   scoringPoses.put(poseType + name, pose);
                }));
        return scoringPoses;
    }

    /**
     * Compiles a list of all scoring poses.
     *
     * @return a list of all scoring poses.
     */
    public static List<Pose2d> getScoringPoses() {
        List<Pose2d> scoringPoses = new ArrayList<>();
        scoringMap.forEach((poseType, poseMap) -> poseMap.forEach((poseName, pose) -> scoringPoses.add(pose)));
        return scoringPoses;
    }

    /**
     * Adds a collecting pose to the config.
     *
     * @param poseType The type of pose to add. For example, "CollectStation".
     * @param poseName The name of the pose to add. For example, "StationCenter".
     * @param pose The pose to add.
     * @return this, for chaining.
     */
    public static void addCollectingPose(String poseType, String poseName, Pose2d pose) {
        collectingMap.putIfAbsent(poseType, new HashMap<>());
        collectingMap.get(poseType).putIfAbsent(poseName, pose);
        if (collectingMap.get(poseType).get(poseName) != pose) {
            throw new IllegalArgumentException("Failed to add collect pose: " + poseName + "/n");
        }
    }

    /**
     * Gets the raw collecting map.
     * Setup as Map<poseType, Map<poseName, Pose2d>>
     *
     * @return the raw collecting map.
     */
    public static Map<String, Map<String, Pose2d>> getRawCollectingMap() {
        return collectingMap;
    }

    /**
     * Compiles a compacted collecting map.
     * Setup as Map<poseName, Pose2d>
     *
     * @return a compacted collecting map.
     */
    public static Map<String, Pose2d> getCollectingMap() {
        Map<String, Pose2d> collectingPoses = new HashMap<>();
        collectingMap.forEach((poseType, poseMap) ->
                poseMap.forEach((name, pose2d) ->
                        collectingPoses.put(poseType + name, pose2d)));
        return collectingPoses;
    }

    /**
     * Compiles a list of all collecting poses.
     *
     * @return a list of all collecting poses.
     */
    public static List<Pose2d> getCollectingPoses() {
        List<Pose2d> collectingPoses = new ArrayList<>();
        collectingMap.forEach((poseType, poseMap) -> poseMap.forEach((poseName, pose) -> collectingPoses.add(pose)));
        return collectingPoses;
    }

    /// Its end of the season, so I just copied my constants here and added them as such.
    private static class PoseConstants {
        public static final Transform2d STATION_OFFSET = new Transform2d(
                Units.inchesToMeters(-17),
                Units.inchesToMeters(0),
                Rotation2d.kZero);
        public static final Transform2d SLOT_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(24), // Added several times to achieve all 5 poses.
                Rotation2d.kZero);
        public static final Transform2d SLOT_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(-30), // Added several times to achieve all 5 poses.
                Rotation2d.kZero);
        public static final Pose2d BARGE_NET_POSE =
                new Pose2d(
                        7.5,
                        2,
                        Rotation2d.fromDegrees(0));
        public static final Pose2d LEFT_STATION_CENTER_POSE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(125.989));
        public static final Pose2d RIGHT_STATION_CENTER_POSE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(234.011));
        public static final Pose2d LEFT_STATION_POSE = LEFT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d RIGHT_STATION_POSE = RIGHT_STATION_CENTER_POSE
                .plus(STATION_OFFSET);
        public static final Pose2d SOUTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(0));
        public static final Pose2d SOUTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(300));
        public static final Pose2d NORTHWEST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(240));
        public static final Pose2d NORTH_FACE_POSE = new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(180));
        public static final Pose2d NORTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(120));
        public static final Pose2d SOUTHEAST_FACE_POSE = new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(60));
        public static final Transform2d BRANCH_OFFSET_LEFT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from the reef.
                Units.inchesToMeters(13 / 2.0), // Offset to left branch.
                Rotation2d.kZero);
        public static final Transform2d BRANCH_OFFSET_RIGHT = new Transform2d(
                Units.inchesToMeters(-28), // Offset away from the reef.
                Units.inchesToMeters(-13 / 2.0), // Offset to the right branch.
                Rotation2d.kZero);
        public static final Pose2d REEF_NORTH_LEFT_POSE =
                NORTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTH_RIGHT_POSE =
                NORTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHEAST_LEFT_POSE =
                NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHEAST_RIGHT_POSE =
                NORTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_NORTHWEST_LEFT_POSE =
                NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_NORTHWEST_RIGHT_POSE =
                NORTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTH_LEFT_POSE =
                SOUTH_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTH_RIGHT_POSE =
                SOUTH_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHEAST_LEFT_POSE =
                SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHEAST_RIGHT_POSE =
                SOUTHEAST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
        public static final Pose2d REEF_SOUTHWEST_LEFT_POSE =
                SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_LEFT);
        public static final Pose2d REEF_SOUTHWEST_RIGHT_POSE =
                SOUTHWEST_FACE_POSE.plus(BRANCH_OFFSET_RIGHT);
    }
}
