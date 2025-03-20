package pedroPathing.examples;

import pedroPathing.TeleOpComp;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;
import pedroPathing.subsystem.IndicatorLight;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp(name="test file")
public class teleOpTest extends LinearOpMode {
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

            if (OpModeTransfer.autoColor == null) {
                leftLight.setColor(IndicatorLight.COLOR_GREEN);
                rightLight.setColor(IndicatorLight.COLOR_GREEN);
            }
            else if (OpModeTransfer.autoColor == TeleOpComp.TeamColor.TEAM_BLUE) {
                //specimenArm.nextClawState();
                leftLight.setColor(IndicatorLight.COLOR_BLUE);
                rightLight.setColor(IndicatorLight.COLOR_BLUE);
            }
            else if (OpModeTransfer.autoColor == TeleOpComp.TeamColor.TEAM_RED) {
                leftLight.setColor(IndicatorLight.COLOR_RED);
                rightLight.setColor(IndicatorLight.COLOR_RED);
            }
            else {
                leftLight.setColor(IndicatorLight.COLOR_YELLOW);
                rightLight.setColor(IndicatorLight.COLOR_YELLOW);
            }
        }


    }
}
