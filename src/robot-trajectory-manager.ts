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
    trim: boolean;
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
  id: number;
  shape: string;
  dimensions: number;
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

interface Request {
  request: string;
  param: unknown;
}

interface Response {
  response: string;
  values: unknown;
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
    return new Promise(res => {
      this._webSocket.send(JSON.stringify(request));
      this._ongoingRequest.push([request, res as (resp: Response) => void]);
    });
  }

  async serverTime(request: TimeRequest): Promise<TimeResponse> {
    return new Promise(res => {
      this._webSocket.send(JSON.stringify(request));
      this._ongoingRequest.push([request, res as (resp: Response) => void]);
    });
  }

  private _ongoingRequest: [Request, (resp: Response) => void][] = [];

  private constructor(private _webSocket: WebSocket) {
    this._webSocket.addEventListener('message', e => this._handleMessage(e));
  }

  private _handleMessage(e: MessageEvent): void {
    const ongoingRequest = this._ongoingRequest.shift();
    if (!ongoingRequest) {
      console.warn('received response when no request is made');
      return;
    }

    const [request, res] = ongoingRequest;
    const resp = JSON.parse(e.data) as Response;
    if (request.request !== resp.response) {
      console.warn('received response for wrong request');
      return;
    }

    if (resp.response === 'trajectory') {
      if (resp.values === null) {
        resp.values = [];
      }
    }
    res(resp);
  }
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
