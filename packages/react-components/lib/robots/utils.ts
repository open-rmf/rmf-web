import type { Task } from 'api-client';
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

//TODO: endpoint should return the equivalent of this
export interface VerboseRobot extends RmfModels.RobotState {
  assignedTasks: Task[];
}

export function makeVerboseRobot(robot: RmfModels.RobotState, assignedTasks: Task[]): VerboseRobot {
  return {
    ...robot,
    assignedTasks: assignedTasks,
  };
}

//TODO: add this functionality into the server
export function allocateTasksToRobots(
  robots: RmfModels.RobotState[],
  tasks: Task[],
): VerboseRobot[] {
  const removableTaskStates = [
    RmfModels.TaskSummary.STATE_ACTIVE,
    RmfModels.TaskSummary.STATE_PENDING,
    RmfModels.TaskSummary.STATE_QUEUED,
  ];
  const robotsWithAssignedTasks = robots.map((robot) => {
    const assignedTasks = tasks.filter((task) => {
      if (
        task.summary.robot_name == robot.name &&
        removableTaskStates.indexOf(task.summary.state) != -1
      ) {
        return task;
      }
    });
    return makeVerboseRobot(robot, assignedTasks);
  });
  return robotsWithAssignedTasks;
}
