package pedroPathing.examples;

import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.*;
@Autonomous(name="test commands")
public class autoTest extends LinearOpMode {
    private SpecimenArm specimenArm = null;
    private CommandScheduler scheduler = null;
    @Override
    public void runOpMode() {
        specimenArm = new SpecimenArm(hardwareMap, telemetry);
        waitForStart();
        scheduler = CommandScheduler.getInstance();
        scheduler.schedule(specimenArm.new doAutoClawStateClose());
        while (opModeIsActive()) {
            scheduler.run();

        }


    }
}
