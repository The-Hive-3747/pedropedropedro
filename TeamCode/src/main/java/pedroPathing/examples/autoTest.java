package pedroPathing.examples;

import android.transition.Slide;

import pedroPathing.TeleOpComp;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;
import pedroPathing.subsystem.IndicatorLight;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Autonomous(name="auto testFILE")
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
                slideArm.new wristGather()
                //slideArm.new IntakeWithSensor()
        );
        while (opModeIsActive()) {
            //scheduler.run();
            IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
            IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");
            leftLight.setColor(IndicatorLight.COLOR_GREEN);



        }
        OpModeTransfer.autoColor = TeleOpComp.TeamColor.TEAM_BLUE;

    }
}
