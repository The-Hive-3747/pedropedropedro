package pedroPathing.examples;

import android.widget.GridLayout;

import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.arcrobotics.ftclib.command.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

@Autonomous(name = "five speci ahh", group = "Examples")
public class fiveSpeciAHHHH extends OpMode {
    private SlideArm slideArm = null;
    private SpecimenArm specimenArm = null;
    public static Pose startPose = new Pose(8,61.5, Math.toRadians(0));

    
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(8, 61.5, Point.CARTESIAN),
                        new Point(35, 70, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pushSamples() {
        return new PathBuilder()
                .addPath(new BezierCurve( // after place, goes to first sample
                        new Point(35, 70, Point.CARTESIAN),
                        new Point(0.7, 43.8, Point.CARTESIAN),
                        new Point(48, 28, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes first sample
                        new Point(48, 28, Point.CARTESIAN),
                        new Point(60, 22, Point.CARTESIAN),
                        new Point(25, 25, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to second sample
                        new Point(25, 25, Point.CARTESIAN),
                        new Point(48, 23, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes second sample
                        new Point(48, 23, Point.CARTESIAN),
                        new Point(60, 10, Point.CARTESIAN),
                        new Point(25, 15, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to third sample
                        new Point(25, 15, Point.CARTESIAN),
                        new Point(48, 16, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes third sample
                        new Point(48, 16, Point.CARTESIAN),
                        new Point(60, 2, Point.CARTESIAN),
                        new Point(20, 5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20,5,Point.CARTESIAN),
                        new Point(35,35, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
            

public void autonomousPathUpdate() {
    switch (pathState) {
        case 0: // Move from start to scoring position
            follower.followPath(preload());
            //specimenArm.goToNextSpecimenState();
            setPathState(1);
            break;

        case 1: // Wait until the robot is near the scoring position
            if (!follower.isBusy()) {
                follower.followPath(pushSamples(), true);
                setPathState(2);
            }
            break;

        case 2: // Wait until the robot is near the scoring position
            if (!follower.isBusy()) {
                setPathState(-1); // End the autonomous routine
            }
            break;
    }
}

public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
}
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        slideArm = new SlideArm(hardwareMap, telemetry);
        specimenArm = new SpecimenArm(hardwareMap, telemetry);
        //buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        slideArm.setWristToReady();

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
/*
    @Override
    public void init () {
        //pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
                follower = new Follower(hardwareMap);
                follower.setStartingPose(startPose);
                //buildPaths();
            }

            @Override
            public void start () {
                schedule(
                        new SequentialCommandGroup(
                                new FollowPath(preload, true)
                new WaitCommand(200).andThen(new FollowPath(pushSamples(), true))
        )
        );
            }
        }

        @Override
        public void stop () {
            CommandScheduler.getInstance().reset();
        }*/
    }