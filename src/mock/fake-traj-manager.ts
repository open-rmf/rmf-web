import {
  RobotTrajectoryManager,
  TimeRequest,
  TimeResponse,
  TrajectoryRequest,
  TrajectoryResponse,
} from '../robot-trajectory-manager';
import trajectories from './data/trajectories.json';

export default class FakeTrajectoryManager implements RobotTrajectoryManager {
  latestTrajectory(request: TrajectoryRequest): Promise<TrajectoryResponse> {
    return trajectories as any;
  }

  serverTime(request: TimeRequest): Promise<TimeResponse> {
    throw new Error('Method not implemented.');
  }
}
