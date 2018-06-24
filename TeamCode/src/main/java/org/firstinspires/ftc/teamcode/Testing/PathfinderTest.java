package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotFunctions.Hardware;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;


@Autonomous
public class PathfinderTest extends LinearOpMode {
    Hardware Robot = new Hardware();
    Pathfinder path = new Pathfinder();

    Waypoint[] points = new Waypoint[]{
            new Waypoint(0, 0, 0),
            new Waypoint(0, 3, 0),
            new Waypoint(-3, 3, 0)
    };

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.01, 1, 4, 60);

    public void runOpMode(){
        waitForStart();

        Trajectory trajectory = Pathfinder.generate(points, config);

        for (int i = 0; i < trajectory.length(); i++) {
            Trajectory.Segment seg = trajectory.get(i);

           telemetry.addData("trajectory","%f,%f,%f,%f,%f,%f,%f,%f\n",
                    seg.dt, seg.x, seg.y, seg.position, seg.velocity,
                    seg.acceleration, seg.jerk, seg.heading);
           telemetry.update();
        }
    }
}
