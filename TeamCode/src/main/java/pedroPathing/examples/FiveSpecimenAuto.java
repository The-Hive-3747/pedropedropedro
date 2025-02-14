package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;

import pedroPathing.TeleOpComp;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.command.*;

import pedroPathing.subsystem.SpecimenArm;

@Autonomous(name = "FiveSpecimenAuto")
public class FiveSpecimenAuto extends LinearOpMode {
    boolean teamChangeRequested = false;
    TeleOpComp.TeamColor HIVE_COLOR = TeleOpComp.TeamColor.TEAM_BLUE;
    public String pathState = null;
    private SlideArm slideArm = null;
    private SpecimenArm specimenArm = null;
    private CommandScheduler scheduler = null;
    public static Pose startPose = new Pose(8,61.5, Math.toRadians(0));

    private Follower follower;

    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(8, 61.5, Point.CARTESIAN),
                        new Point(39, 75, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pushSamples() {
        return new PathBuilder()
                .addPath(new BezierCurve( // after place, goes to first sample
                        new Point(39, 75, Point.CARTESIAN),
                        new Point(0.7, 43.8, Point.CARTESIAN),
                        new Point(54, 25, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // pushes first sample
                        new Point(54, 25, Point.CARTESIAN),
                        new Point(25, 20, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to second sample
                        new Point(25, 20, Point.CARTESIAN),
                        new Point(48, 20, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes second sample
                        new Point(48, 20, Point.CARTESIAN),
                        new Point(60, 10, Point.CARTESIAN),
                        new Point(25, 12, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to third sample
                        new Point(25, 12, Point.CARTESIAN),
                        new Point(48, 14, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes third sample
                        new Point(48, 14, Point.CARTESIAN),
                        new Point(70, 0, Point.CARTESIAN),
                        new Point(20, 2, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20,2,Point.CARTESIAN),
                        new Point(12,5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score1Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(12, 5, Point.CARTESIAN),
                        new Point(12, 70, Point.CARTESIAN),
                        new Point(39, 73, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }
    public static PathChain pickup2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(39, 73, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(13, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain score2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(13, 39, Point.CARTESIAN),
                        new Point(0, 60, Point.CARTESIAN),
                        new Point(40.5, 71, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(40.5, 71, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(14, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(14, 42, Point.CARTESIAN),
                        new Point(0, 60, Point.CARTESIAN),
                        new Point(40.5, 69, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(40.5, 69, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(14, 41, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(14, 41, Point.CARTESIAN),
                        new Point(0, 60, Point.CARTESIAN),
                        new Point(40.5, 67, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public class FollowPreload extends CommandBase {
        boolean followerStarted = false;
        public FollowPreload() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(preload());
                pathState = "score preload";

                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowPushSamples extends CommandBase {
        boolean followerStarted = false;
        public FollowPushSamples() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pushSamples());
                pathState = "pushSamples";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore1Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore1Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score1Specimen());
                pathState = "score 2nd speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup2Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowPickup2Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup2Specimen());
                pathState = "pickup 3rd speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowScore2Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore2Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score2Specimen());
                pathState = "score 3rd speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup3Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowPickup3Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup3Specimen());
                pathState = "pickup 4th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore3Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore3Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score3Specimen());
                pathState = "score 4th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup4Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowPickup4Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup4Specimen());
                pathState = "pickup 5th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore4Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore4Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score4Specimen());
                pathState = "score 5th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        slideArm = new SlideArm(hardwareMap, telemetry, true);
        specimenArm = new SpecimenArm(hardwareMap, telemetry, true);


        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");
        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right && !teamChangeRequested) {
                teamChangeRequested = true;
                if (HIVE_COLOR == TeleOpComp.TeamColor.TEAM_BLUE) {
                    HIVE_COLOR = TeleOpComp.TeamColor.TEAM_RED;
                    OpModeTransfer.autoColor = TeleOpComp.TeamColor.TEAM_RED;
                } else {
                    HIVE_COLOR = TeleOpComp.TeamColor.TEAM_BLUE;
                    OpModeTransfer.autoColor = TeleOpComp.TeamColor.TEAM_BLUE;
                }
            }
            if (!gamepad1.dpad_right && teamChangeRequested) {
                teamChangeRequested = false;
            }
            if (HIVE_COLOR == TeleOpComp.TeamColor.TEAM_BLUE) {
                leftLight.setColor(IndicatorLight.COLOR_BLUE);
                rightLight.setColor(IndicatorLight.COLOR_SAGE);
            } else {
                leftLight.setColor(IndicatorLight.COLOR_RED);
                rightLight.setColor(IndicatorLight.COLOR_SAGE);
            }
        }

        waitForStart();
        scheduler = CommandScheduler.getInstance();
        slideArm.slideBrakes();
        leftLight.setColor(IndicatorLight.COLOR_AZURE);
        rightLight.setColor(IndicatorLight.COLOR_AZURE);
        scheduler.schedule(
                new SequentialCommandGroup(
                        slideArm.new wristStow(),
                        specimenArm.new doAutoClawStateClose(),
                        specimenArm.new SpecimenArmNextState(),
                        this.new FollowPreload(),
                        specimenArm.new SpecimenArmNextState(),
                        new WaitCommand(80),
                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmNextState()
                                .alongWith(
                        this.new FollowPushSamples()),
                        specimenArm.new doAutoClawStateClose(),
                        specimenArm.new SpecimenArmNextState()
                                .alongWith(
                        this.new FollowScore1Specimen()),
                        specimenArm.new SpecimenArmNextState(),
                        new WaitCommand(80),
                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmNextState(),
                        this.new FollowPickup2Specimen(),
                        specimenArm.new doAutoClawStateClose()
                                .andThen(
                                        specimenArm.new SpecimenArmNextState()
                                ),
                        this.new FollowScore2Specimen(),
                        specimenArm.new SpecimenArmNextState()
                                .andThen(
                                        new WaitCommand(80)
                                                .andThen(
                                        specimenArm.new doAutoClawStateOpen()
                                                )
                                ),
                        specimenArm.new SpecimenArmNextState(),
                        this.new FollowPickup3Specimen(),
                        specimenArm.new doAutoClawStateClose()
                                .andThen(
                                        specimenArm.new SpecimenArmNextState()
                                ),
                        this.new FollowScore3Specimen(),
                        specimenArm.new SpecimenArmNextState()
                                .andThen(
                                        new WaitCommand(80)
                                                .andThen(
                                                specimenArm.new doAutoClawStateOpen()
                                                )
                                ),
                        specimenArm.new SpecimenArmNextState(),
                        this.new FollowPickup4Specimen(),
                        specimenArm.new doAutoClawStateClose()
                                .andThen(
                                        specimenArm.new SpecimenArmNextState()
                                ),
                        this.new FollowScore4Specimen(),
                        specimenArm.new SpecimenArmNextState()
                                .andThen(
                                        new WaitCommand(80)
                                                .andThen(
                                                        specimenArm.new doAutoClawStateOpen()
                                                )
                                ),
                        specimenArm.new SpecimenArmNextState()
                )
       );
        while (opModeIsActive()) {
            follower.update();

            // Feedback to Driver Hub
            telemetry.addData("path state",  pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
            scheduler.run();


        }

    }
    }