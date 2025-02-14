package pedroPathing.examples;

import android.widget.GridLayout;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.SlideArm;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
//import com.pedropathing.commands.FollowPath;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

@Autonomous(name = "five speci ahh", group = "Examples")
public class fiveSpeciAHHHH extends LinearOpMode {
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
                //.setZeroPowerAccelerationMultiplier(2)
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
                //.setZeroPowerAccelerationMultiplier(2)
                .build();
    }
    public static PathChain pickup1Specimen() {
        return new PathBuilder()
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
                        new Point(39, 73.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }
    public static PathChain goTo2Specimen() {
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
    public static PathChain pickup2Specimen() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(13, 42, Point.CARTESIAN),
                        new Point(13, 39, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain score2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(13, 39, Point.CARTESIAN),
                        new Point(0, 60, Point.CARTESIAN),
                        new Point(40, 71, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(40, 71, Point.CARTESIAN),
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
                        new Point(40, 69, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(40, 69, Point.CARTESIAN),
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
                        new Point(41, 67, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public class FollowPreload extends CommandBase {
        //boolean isDone = false;
        boolean followerStarted = false;
        public FollowPreload() {}
        @Override
        public void initialize() {
            //isDone = false;
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(preload());
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowPushSamples extends CommandBase {
        //boolean isDone = false;
        boolean followerStarted = false;
        public FollowPushSamples() {}
        @Override
        public void initialize() {
            //isDone = false;
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pushSamples());
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowPickup1Specimen extends CommandBase {
        //boolean isDone = false;
        boolean followerStarted = false;
        public FollowPickup1Specimen() {}
        @Override
        public void initialize() {
            //isDone = false;
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup1Specimen());
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }


    public class FollowScore1Specimen extends CommandBase {
        //boolean isDone = false;
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
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowGoTo2Specimen extends CommandBase {
        //boolean isDone = false;
        boolean followerStarted = false;
        public FollowGoTo2Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(goTo2Specimen());
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup2Specimen extends CommandBase {
        //boolean isDone = false;
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
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore2Specimen extends CommandBase {
        //boolean isDone = false;
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
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup3Specimen extends CommandBase {
        //boolean isDone = false;
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
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore3Specimen extends CommandBase {
        //boolean isDone = false;
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
        specimenArm = new SpecimenArm(hardwareMap, telemetry);
        waitForStart();
        scheduler = CommandScheduler.getInstance();
        slideArm.slideBrakes();
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
                        this.new FollowGoTo2Specimen(),
                        //this.new FollowPickup2Specimen(),
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
            telemetry.addData("path state", "this is useless but i dont wanna get rid of it cuz it should be implemented in the future. guys ive been here for 4 hours i acc cannot think :)");
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
            scheduler.run();


        }

    }
    }