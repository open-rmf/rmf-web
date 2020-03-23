import {
  RobotTrajectoryManager,
  TimeRequest,
  TimeResponse,
  TrajectoryRequest,
  TrajectoryResponse,
} from '../robot-trajectory-manager';
import trajectories from './data/trajectories.json';

export default class FakeTrajectoryManager implements RobotTrajectoryManager {
  async latestTrajectory(request: TrajectoryRequest): Promise<TrajectoryResponse> {
    if (request.param.map_name === 'L1') {
      return trajectories as any;
    }
    return {
      response: 'trajectory',
      values: [],
    };
  }

  serverTime(request: TimeRequest): Promise<TimeResponse> {
    throw new Error('Method not implemented.');
  }
}
