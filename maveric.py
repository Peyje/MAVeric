import click
import csv
import trajectory_planner
import draw as visualize
from numpy import *


@click.command()
@click.option('--draw/--nodraw', default=True, help='Use to visualize the trajectories with matplotlib.')
@click.option('--eval/--noeval', default=False, help='Output evaluated trajectory instead of coefficients.')
@click.option('--evaltime', default=20, help='Number of samples per second. Default: 20')
@click.argument('input_path', type=click.Path(exists=True))
@click.argument('output_path', type=click.File('w'))
def generate(input_path, output_path, draw, eval, evaltime):
    """
    MAVeric calculates a minimum snap trajectory for the given waypoints in INPUT. It then stores this trajectory in
    OUTPUT. See the options for further details.

    The input file has to be a CSV file, where every new line is a new waypoint. A waypoint consists of a x, y, z,
    and yaw value.

    Printing to the standard output instead of an output file can be achieved by simply giving '-' as the OUTPUT.

    Run 'python maveric.py waypoints_example.txt -' for an example.
    """
    click.secho('\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~', fg='blue')
    click.secho('This is MAVeric.\n', fg='green')
    click.secho('Loading the the waypoints from the input path:', fg='green')

    # gather waypoints from input file
    waypoints_cli = []
    with open(input_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            waypoints_cli.append(row)

    # be verbose and show those waypoints
    for i in range(len(waypoints_cli)):
        click.echo(waypoints_cli[i])

    click.secho('\nCalculate the trajectory coefficients:', fg='green')

    # calculate trajectory
    waypoints, trajectory = trajectory_planner.planner(waypoints_cli)

    # visualize the trajectory
    if draw:
        click.secho('\nInvoking visualization with matplotlib..', fg='green')
        visualize.draw_traj(waypoints, trajectory)

    click.secho('\nPrint to given output..', fg='green')

    # print to output
    if not eval:
        # just print the coefficients
        for i in range(len(trajectory)):
            output_path.write('TRAJECTORY FROM WAYPOINT %d TO %d:\n' % (i, i+1))

            for j in range(len(trajectory[i])):
                if j == 0:
                    output_path.write('Coefficients for x trajectory are:\n')
                if j == 5:
                    output_path.write('\nCoefficients for y trajectory are:\n')
                if j == 10:
                    output_path.write('\nCoefficients for z trajectory are:\n')
                if j == 15:
                    output_path.write('\nCoefficients for yaw trajectory are:\n')

                output_path.write('%s ' % trajectory[i][j])
            output_path.write('\n\n')

    else:
        # OR print the evaluated trajectory
        output_path.write('Evaluated trajectory: time, x, y, z, phi\n\n')

        for i in range(len(trajectory)):
            output_path.write('TRAJECTORY FROM WAYPOINT %d TO %d:\n' % (i, i+1))

            t = linspace(waypoints[i].time, waypoints[i + 1].time, (waypoints[i + 1].time - waypoints[i].time) * evaltime)
            x_path = (trajectory[i][0] * t ** 4 + trajectory[i][1] * t ** 3 + trajectory[i][2] * t ** 2 + trajectory[i][3] * t + trajectory[i][4])
            y_path = (trajectory[i][5] * t ** 4 + trajectory[i][6] * t ** 3 + trajectory[i][7] * t ** 2 + trajectory[i][8] * t + trajectory[i][9])
            z_path = (trajectory[i][10] * t ** 4 + trajectory[i][11] * t ** 3 + trajectory[i][12] * t ** 2 + trajectory[i][13] * t + trajectory[i][14])
            yaw_path = (trajectory[i][15] * t ** 2 + trajectory[i][16] * t + trajectory[i][17])

            for j in range(len(t)):
                output_path.write('%f, %f, %f, %f, %f\n' % (t[j], x_path[j], y_path[j], z_path[j], yaw_path[j]))
            output_path.write('\n\n')

    click.secho('\nDone. Bye!', fg='green')
    click.secho('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n', fg='blue')


if __name__ == '__main__':
    generate()
