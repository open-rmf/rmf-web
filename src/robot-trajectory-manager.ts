import { Knot } from './util/cublic-spline';

// RawVelocity received from server is in this format (x, y, theta)
export type RawVelocity = [number, number, number];

// RawPose2D received from server is in this format (x, y, theta)
export type RawPose2D = [number, number, number];

interface RawKnot {
  t: number; // milliseconds
  v: RawVelocity;
  x: RawPose2D;
}

export interface TrajectoryRequest {
  request: 'trajectory';
  param: {
    map_name: string;
    duration: number;
  };
}

export interface TimeRequest {
  request: 'time';
  param: {};
}

export interface TimeResponse {
  response: 'time';
  values: [number];
}

export interface Trajectory {
  shape: string;
  dimensions: number[];
  segments: RawKnot[];
}

export interface TrajectoryResponse {
  response: 'trajectory';
  values: Trajectory[];
}

export interface RobotTrajectoryManager {
  serverTime(request: TimeRequest): Promise<TimeResponse>;
  latestTrajectory(request: TrajectoryRequest): Promise<TrajectoryResponse>;
}

export class DefaultTrajectoryManager {
  static async create(url: string): Promise<DefaultTrajectoryManager> {
    const ws = new WebSocket(url);
    await new Promise((res, rej) => {
      ws.addEventListener('open', function listener() {
        ws.removeEventListener('open', listener);
        res();
      });

      ws.addEventListener('error', function listener(e) {
        ws.removeEventListener('error', listener);
        rej(e);
      });
    });
    return new DefaultTrajectoryManager(ws);
  }

  async latestTrajectory(request: TrajectoryRequest): Promise<TrajectoryResponse> {
    if (this._ongoingRequest['trajectory']) {
      throw new Error('only one request can be sent at once');
    }
    this._ongoingRequest['trajectory'] = true;
    this._webSocket.send(JSON.stringify(request));
    return new Promise(res => {
      const listener = (ev: MessageEvent) => {
        this._webSocket.removeEventListener('message', listener);
        delete this._ongoingRequest['trajectory'];
        res(JSON.parse(ev.data));
      };
      this._webSocket.addEventListener('message', listener);
    });
  }

  async serverTime(request: TimeRequest): Promise<TimeResponse> {
    if (this._ongoingRequest['time']) {
      throw new Error('only one request can be sent at once');
    }
    this._ongoingRequest['time'] = true;
    this._webSocket.send(JSON.stringify(request));
    return new Promise(res => {
      const listener = (ev: MessageEvent) => {
        this._webSocket.removeEventListener('message', listener);
        delete this._ongoingRequest['time'];
        res(JSON.parse(ev.data));
      };
      this._webSocket.addEventListener('message', listener);
    });
  }

  private _ongoingRequest: Record<string, boolean> = {};

  private constructor(private _webSocket: WebSocket) {}
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
