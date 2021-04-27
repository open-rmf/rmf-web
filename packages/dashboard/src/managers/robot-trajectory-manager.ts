import { Knot } from '../util/cublic-spline';
import TrajectorySocketManager from './trajectory-socket-manager';

// RawVelocity received from server is in this format (x, y, theta)
export type RawVelocity = [number, number, number];

// RawPose2D received from server is in this format (x, y, theta)
export type RawPose2D = [number, number, number];

export interface RawKnot {
  t: number; // milliseconds
  v: RawVelocity;
  x: RawPose2D;
}

export interface TrajectoryRequest {
  request: 'trajectory';
  param: {
    map_name: string;
    duration: number;
    trim: boolean;
  };
  token?: string;
}

export interface TimeRequest {
  request: 'time';
  param: {};
  token?: string;
}

export interface TimeResponse {
  response: 'time';
  values: [number];
}

export interface Trajectory {
  id: number;
  shape: string;
  dimensions: number;
  segments: RawKnot[];
  robot_name: string;
  fleet_name: string;
  map_name: string;
}

export interface TrajectoryResponse {
  response: 'trajectory';
  values: Trajectory[];
  conflicts: Conflict[];
  error?: string;
}

export type Conflict = number[];

export interface RobotTrajectoryManager {
  serverTime(request: TimeRequest): Promise<TimeResponse>;
  latestTrajectory(request: TrajectoryRequest): Promise<TrajectoryResponse>;
}

interface Request {
  request: string;
  param: unknown;
}

interface Response {
  response: string;
  values: unknown;
  error?: string;
}

export class DefaultTrajectoryManager extends TrajectorySocketManager {
  constructor(ws: WebSocket) {
    super();
    if (ws) this._webSocket = ws;
  }

  async latestTrajectory(request: TrajectoryRequest): Promise<TrajectoryResponse> {
    const event = await this._send(JSON.stringify(request), this._webSocket);
    const resp = JSON.parse(event.data);
    this._checkResponse(request, resp);
    if (resp.values === null) {
      resp.values = [];
    }
    return resp as TrajectoryResponse;
  }

  async serverTime(request: TimeRequest): Promise<TimeResponse> {
    const event = await this._send(JSON.stringify(request), this._webSocket);
    const resp = JSON.parse(event.data);
    this._checkResponse(request, resp);
    return resp as TimeResponse;
  }

  static getRobotNameFromPathId(
    pathId: number,
    trajectories: readonly Trajectory[],
  ): string | undefined {
    const traj = trajectories.find((trajectory) => trajectory.id === pathId);
    return traj?.robot_name;
  }

  private _checkResponse(request: Request, resp: Response): void {
    if (resp.error) {
      if (resp.error !== 'token expired') throw new Error(resp.error);
    } else if (request.request !== resp.response) {
      console.warn(
        `received response for wrong request. Request: ${request.request} Response: ${resp.response}`,
      );
      throw new Error('received response for wrong request');
    }
  }

  private _webSocket: WebSocket | undefined;
}

export function rawKnotsToKnots(rawKnots: RawKnot[]): Knot[] {
  const knots: Knot[] = [];

  for (const rawKnot of rawKnots) {
    const [poseX, poseY, poseTheta] = rawKnot.x;
    const [velocityX, velocityY, velocityTheta] = rawKnot.v;
    knots.push({
      pose: {
        x: poseX,
        y: poseY,
        theta: poseTheta,
      },
      velocity: {
        x: velocityX,
        y: velocityY,
        theta: velocityTheta,
      },
      time: rawKnot.t,
    });
  }

  return knots;
}
