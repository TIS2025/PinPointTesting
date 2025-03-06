package org.firstinspires.ftc.teamcode.VisionUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "LL" ,name = "Limelight Position Test")
public class LimePosTest extends LinearOpMode {

    FtcDashboard dashboard;

    LimeLightUtils limeUtil;

    @Override
    public void runOpMode() throws InterruptedException {
        limeUtil=new LimeLightUtils(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry.setMsTransmissionInterval(100);
        limeUtil.limelight3A.pipelineSwitch(0);
        waitForStart();
        limeUtil.limelight3A.start();
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());

        while (opModeIsActive()) {

            limeUtil.clear_samples();
            limeUtil.filter(limeUtil.limelight3A.getLatestResult(),telemetry);
            limeUtil.print_samples(telemetry);
            telemetry.update();
        }
    }
}

