package org.ironmaple.simulation.seasonspecific.reefscape2025.opponentsim;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.opponentsim.pathfinding.MapleADStar;

import java.util.ArrayList;
import java.util.List;

public class ReefscapeOpponentManager extends org.ironmaple.simulation.opponentsim.OpponentManager {


    /**
     * MapleSim Opponent sim currently relies on Pathplanner with a modified pathfinder.
     * This is to be changed soon. ^TM
     */
    public ReefscapeOpponentManager() {
        super();
    }

    public Pair<Pose2d, String> getNextAlgaeScoreTarget(DriverStation.Alliance alliance, int id) {
        boolean targetExists = false;
        // Target = targetArray.get(rand()[0-1] * targetArray.size() - 1); // Array values start at 0, size starts at 1, subtract 1.
        Pose2d target = ReefscapeManagerConstants.TARGET_BARGE_POSES.get(((int) Math.round(Math.random() * (ReefscapeManagerConstants.TARGET_BARGE_POSES.size() - 1))));
        for (Pair<Pose2d, Integer> pair : robotTargets) {
            if (pair.getFirst().equals(target)) {
                targetExists = true;
                break;
            }
        }
        if (targetExists) {
            return getNextAlgaeScoreTarget(alliance, id);
        } else {
            robotTargets.removeIf(pair -> pair.getSecond() == id);
        }
        robotTargets.add(new Pair<>(target, id));
        return Pair.of(target, "Algae");
    }

    public Pair<Pose2d, String> getNextCoralScoreTarget(DriverStation.Alliance alliance, int id) {
        boolean targetExists = false;
        // Target = targetArray.get(rand()[0-1] * targetArray.size() - 1); // Array values start at 0, size starts at 1, subtract 1.
        Pose2d target = ReefscapeManagerConstants.TARGET_REEF_POSES.get(((int) Math.round(Math.random() * (ReefscapeManagerConstants.TARGET_REEF_POSES.size() - 1))));
        for (Pair<Pose2d, Integer> pair : robotTargets) {
            if (pair.getFirst().equals(target)) {
                targetExists = true;
                break;
            }
        }
        if (targetExists) {
            return getNextCoralScoreTarget(alliance, id);
        } else {
            robotTargets.removeIf(pair -> pair.getSecond() == id);
        }
        robotTargets.add(new Pair<>(target, id));
        return Pair.of(target, "Coral");
    }

    /**
     *
     * @param alliance
     * @param id
     * @param scoring [0-2] 0: Score any, 1: Score Coral Only, 2: Score Algae Only[Barge only]
     * @return
     */
    public Pair<Pose2d, String> getNextScoreTarget(DriverStation.Alliance alliance, int id, int scoring) {
        if (scoring == 0) {
            return Math.random() > 0.8 ? getNextAlgaeScoreTarget(alliance, id) : getNextCoralScoreTarget(alliance, id);
        } else if (scoring == 1) {
            return getNextCoralScoreTarget(alliance, id);
        } else if (scoring == 2) {
            return getNextAlgaeScoreTarget(alliance, id);
        } else {
            return getNextScoreTarget(alliance, id);
        }
    }

    @Override
    public Pair<Pose2d, String> getNextScoreTarget(DriverStation.Alliance alliance, int id) {
        return getNextScoreTarget(alliance, id, 0);
    }

    @Override
    public Pair<Pose2d, String> getNextCollectTarget(DriverStation.Alliance alliance, int id) {
        boolean targetExists = false;
        // Target = targetArray.get(rand()[0-1] * targetArray.size() - 1); // Array values start at 0, size starts at 1, subtract 1.
        Pose2d target = ReefscapeManagerConstants.STATION_POSES.get(((int) Math.round(Math.random() * (ReefscapeManagerConstants.STATION_POSES.size() - 1))));
            for (Pair<Pose2d, Integer> pair : robotTargets) {
            if (pair.getFirst().equals(target)) {
                targetExists = true;
                break;
            }
        }
        if (targetExists) {
            return getNextCollectTarget(alliance, id);
        } else {
            robotTargets.removeIf(pair -> pair.getSecond() == id);
        }
        robotTargets.add(new Pair<>(target, id));
        return Pair.of(target, "Station"); // Only Station
    }

    public static class ReefscapeManagerConstants extends ManagerConstants {

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

        private static final List<Pose2d> TARGET_REEF_POSES = new ArrayList<>();
        private static final List<Pose2d> TARGET_BARGE_POSES = new ArrayList<>();
        private static final List<Pose2d> STATION_POSES = new ArrayList<>();
        static {
            TARGET_REEF_POSES.add(0, REEF_SOUTH_LEFT_POSE);
            TARGET_REEF_POSES.add(1, REEF_SOUTH_RIGHT_POSE);
            TARGET_REEF_POSES.add(2, REEF_SOUTHEAST_LEFT_POSE);
            TARGET_REEF_POSES.add(3, REEF_SOUTHEAST_RIGHT_POSE);
            TARGET_REEF_POSES.add(4, REEF_NORTHEAST_LEFT_POSE);
            TARGET_REEF_POSES.add(5, REEF_NORTHEAST_RIGHT_POSE);
            TARGET_REEF_POSES.add(6, REEF_NORTH_LEFT_POSE);
            TARGET_REEF_POSES.add(7, REEF_NORTH_RIGHT_POSE);
            TARGET_REEF_POSES.add(8, REEF_NORTHWEST_LEFT_POSE);
            TARGET_REEF_POSES.add(9, REEF_NORTHWEST_RIGHT_POSE);
            TARGET_REEF_POSES.add(10, REEF_SOUTHWEST_LEFT_POSE);
            TARGET_REEF_POSES.add(11, REEF_SOUTHWEST_RIGHT_POSE);

            TARGET_BARGE_POSES.add(0, BARGE_NET_POSE);
            TARGET_BARGE_POSES.add(1, BARGE_NET_POSE.plus(SLOT_OFFSET_LEFT));
            TARGET_BARGE_POSES.add(2, BARGE_NET_POSE.plus(SLOT_OFFSET_RIGHT));

            STATION_POSES.add(0, LEFT_STATION_POSE);
            STATION_POSES.add(1, LEFT_STATION_POSE.plus(SLOT_OFFSET_LEFT));
            STATION_POSES.add(2, LEFT_STATION_POSE.plus(SLOT_OFFSET_RIGHT));
            STATION_POSES.add(3, RIGHT_STATION_POSE);
            STATION_POSES.add(4, RIGHT_STATION_POSE.plus(SLOT_OFFSET_LEFT));
            STATION_POSES.add(5, RIGHT_STATION_POSE.plus(SLOT_OFFSET_RIGHT));
        }

    }
}
