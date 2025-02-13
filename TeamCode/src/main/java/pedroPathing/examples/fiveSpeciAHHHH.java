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
                        new Point(41, 70, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2)
                .build();
    }
    public static PathChain pushSamples() {
        return new PathBuilder()
                .addPath(new BezierCurve( // after place, goes to first sample
                        new Point(35, 70, Point.CARTESIAN),
                        new Point(0.7, 43.8, Point.CARTESIAN),
                        new Point(48, 28, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // pushes first sample
                        new Point(48, 28, Point.CARTESIAN),
                        new Point(25, 23, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to second sample
                        new Point(25, 23, Point.CARTESIAN),
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
                        new Point(20, 0, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20,0,Point.CARTESIAN),
                        new Point(15,5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2)
                .build();
    }
    public static PathChain score1Sample() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15, 5, Point.CARTESIAN),
                        new Point(12, 70, Point.CARTESIAN),
                        new Point(41, 69, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }
    public static PathChain pickup2Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(41, 69, Point.CARTESIAN),
                        new Point(12, 41, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain place2Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(12, 41, Point.CARTESIAN),
                        new Point(41, 68, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup3Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(41, 68, Point.CARTESIAN),
                        new Point(12, 41, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain place3Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(12, 41, Point.CARTESIAN),
                        new Point(41, 67, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup4Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(41, 67, Point.CARTESIAN),
                        new Point(12, 41, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain place4Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(12, 41, Point.CARTESIAN),
                        new Point(41, 66, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public class FollowPreloadPath extends CommandBase {
        boolean isDone = false;
        public FollowPreloadPath() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            follower.followPath(preload());
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class FollowPushSamplesPath extends CommandBase {
        boolean isDone = false;
        public FollowPushSamplesPath() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            follower.followPath(pushSamples());
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class FollowScore1SamplePath extends CommandBase {
        boolean isDone = false;
        public FollowScore1SamplePath() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            follower.followPath(score1Sample());
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }




    /** This method is called continuously after Init while waiting for "play". **/


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        slideArm = new SlideArm(hardwareMap, telemetry, true);
        specimenArm = new SpecimenArm(hardwareMap, telemetry);
        waitForStart();
        scheduler = CommandScheduler.getInstance();
        scheduler.schedule(
                slideArm.new wristStow(),
                this.new FollowPreloadPath()
                        .andThen(
                                new ParallelCommandGroup(
                                        specimenArm.new SpecimenArmNextState(),
                                        specimenArm.new doAutoClawStateOpen()
                                )
                        ),
                this.new FollowPushSamplesPath(),
                specimenArm.new doAutoClawStateClose(),
                specimenArm.new SpecimenArmNextState(),
                this.new FollowScore1SamplePath(),
                specimenArm.new SpecimenArmNextState(),
                specimenArm.new doAutoClawStateOpen()
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