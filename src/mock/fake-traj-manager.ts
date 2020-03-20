import Big from 'big.js';
import {
  RobotTrajectoryManager,
  TimeRequest,
  TimeResponse,
  Trajectory,
  TrajectoryRequest,
  TrajectoryResponse,
} from '../robot-trajectory-manager';
import trajectories from './data/trajectories.json';

export default class FakeTrajectoryManager implements RobotTrajectoryManager {
  trajectory(request: TrajectoryRequest): Promise<TrajectoryResponse> {
    throw new Error('Method not implemented.');
  }

  serverTime(request: TimeRequest): Promise<TimeResponse> {
    throw new Error('Method not implemented.');
  }

  latestTrajectory(period: Big): Promise<Trajectory[]> {
    return trajectories as any;
  }
}
