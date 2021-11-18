package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.HardwareRagnarok;

@Autonomous(name="--Alpha-- Auto V1", group="Ragnarok")
public class Auto_V1_Blue extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot   = new HardwareRagnarok();
        robot.init(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,63, -Math.PI/2));

        waitForStart();

        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-60, 0), Math.PI)
                .splineTo(new Vector2d(0, -60), Math.PI/2)
                .build();

        drive.followTrajectory(trajectory);
    }
}
