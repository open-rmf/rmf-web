import * as RmfModels from 'rmf-models';

/**
 * Returns a uniquely identifiable string representing a robot.
 */
export function robotHash(name: string, fleet: string): string {
  return `${name}__${fleet}`;
}

export function robotModeToString(robotMode: RmfModels.RobotMode): string {
  switch (robotMode.mode) {
    case RmfModels.RobotMode.MODE_CHARGING:
      return 'Charging';
    case RmfModels.RobotMode.MODE_DOCKING:
      return 'Docking';
    case RmfModels.RobotMode.MODE_EMERGENCY:
      return 'Emergency';
    case RmfModels.RobotMode.MODE_GOING_HOME:
      return 'Going Home';
    case RmfModels.RobotMode.MODE_IDLE:
      return 'Idle';
    case RmfModels.RobotMode.MODE_MOVING:
      return 'Moving';
    case RmfModels.RobotMode.MODE_PAUSED:
      return 'Paused';
    case RmfModels.RobotMode.MODE_WAITING:
      return 'Waiting';
    default:
      return `Unknown (${robotMode.mode})`;
  }
}

export interface VerboseRobot {
  name: string;
  model: string;
  task_id: string;
  seq: number;
  mode: RmfModels.RobotMode;
  battery_percent: number;
  location: RmfModels.Location;
  path: RmfModels.Location[];
  assigned_tasks: RmfModels.TaskSummary[];
}

export function makeVerboseRobot(
  robot: RmfModels.RobotState,
  assignedTasks: RmfModels.TaskSummary[],
): VerboseRobot {
  const { name, model, task_id, seq, mode, battery_percent, location, path } = robot;
  return {
    name,
    model,
    task_id,
    seq,
    mode,
    battery_percent,
    location,
    path,
    assigned_tasks: assignedTasks,
  };
}

export function allocateTasksToRobots(
  robots: RmfModels.RobotState[],
  tasks: RmfModels.TaskSummary[],
): VerboseRobot[] {
  const removableTaskStates = [
    RmfModels.TaskSummary.STATE_ACTIVE,
    RmfModels.TaskSummary.STATE_PENDING,
    RmfModels.TaskSummary.STATE_QUEUED,
  ];
  const robotsWithAssignedTasks = robots.map((robot) => {
    const assignedTasks = tasks.filter((task) => {
      if (task.robot_name == robot.name && removableTaskStates.indexOf(task.state) != -1) {
        return task;
      }
    });
    return makeVerboseRobot(robot, assignedTasks);
  });
  return robotsWithAssignedTasks;
}
