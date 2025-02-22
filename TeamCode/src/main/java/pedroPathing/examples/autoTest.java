package pedroPathing.examples;

import android.transition.Slide;

import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@Autonomous(name="test file")
public class autoTest extends LinearOpMode {
    private SpecimenArm specimenArm = null;
    private CommandScheduler scheduler = null;
    private SlideArm slideArm = null;
    @Override
    public void runOpMode() {
        specimenArm = new SpecimenArm(hardwareMap, telemetry, true);
        slideArm = new SlideArm(hardwareMap, telemetry, true);
        waitForStart();
        scheduler = CommandScheduler.getInstance();
        scheduler.schedule(
                slideArm.new wristGather(),
                slideArm.new IntakeWithSensor()
        );
        while (opModeIsActive()) {
            scheduler.run();
            /*
            if (gamepad1.y) {
                specimenArm.nextClawState();
            }*/

        }


    }
}
