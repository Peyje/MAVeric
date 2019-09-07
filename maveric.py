import click
import csv
import trajectory_planner
import draw as visualize


@click.command()
@click.option('--draw/--nodraw', default=True, help='Use to visualize the trajectories with matplotlib.')
@click.argument('input_path', type=click.Path(exists=True))
@click.argument('output_path', type=click.File('wb'))
def generate(input_path, output_path, draw):
    """MAVeric calculates a minimum snap trajectory for the given waypoints in INPUT. It then stores this trajectory in
    OUTPUT. See the options for further details."""
    click.echo()
    click.secho('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~', fg='blue')
    click.secho('This is MAVeric.', fg='green')
    click.echo()
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

    click.echo()
    click.secho('Calculate the trajectory coefficients:', fg='green')

    # calculate trajectory
    waypoints, trajectory = trajectory_planner.planner(waypoints_cli)

    # visualize the trajectory
    if draw:
        click.echo()
        click.secho('Invoking visualization with matplotlib..', fg='green')
        visualize.draw_traj(waypoints, trajectory)

    click.secho('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~', fg='blue')
    click.echo()


if __name__ == '__main__':
    generate()
