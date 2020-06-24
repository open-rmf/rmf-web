import { Knot } from './util/cublic-spline';
import { TrajectoryCoords } from './components/schedule-visualizer/robot-trajectories-overlay';

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
  conflicts: Conflict[];
}

export type Conflict = number;

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
    const event = await this._send(JSON.stringify(request));
    const resp = JSON.parse(event.data);
    this._checkResponse(request, resp);
    if (resp.values === null) {
      resp.values = [];
    }
    return resp as TrajectoryResponse;
  }

  async serverTime(request: TimeRequest): Promise<TimeResponse> {
    const event = await this._send(JSON.stringify(request));
    const resp = JSON.parse(event.data);
    this._checkResponse(request, resp);
    return resp as TimeResponse;
  }

  private _ongoingRequest: Promise<MessageEvent> | null = null;

  private constructor(private _webSocket: WebSocket) { }

  private _listenOnce<K extends keyof WebSocketEventMap>(
    event: K,
    listener: (e: WebSocketEventMap[K]) => unknown,
  ): void {
    this._webSocket.addEventListener(event, e => {
      this._webSocket.removeEventListener(event, listener);
      listener(e);
    });
  }

  /**
   * Sends a message and waits for response from the server.
   *
   * @remarks This is an alternative to the old implementation of creating a promise, storing the
   * resolver and processing each message in an event loop. Advantage of this is that each message
   * processing logic can be self-contained without a need for a switch or if elses.
   */
  private async _send(payload: WebSocketSendParam0T): Promise<MessageEvent> {
    // response should come in the order that requests are sent, this should allow multiple messages
    // in-flight while processing the responses in the order they are sent.
    this._webSocket.send(payload);
    // waits for the earlier response to be processed.
    if (this._ongoingRequest) {
      await this._ongoingRequest;
    }

    this._ongoingRequest = new Promise(res => {
      this._listenOnce('message', e => {
        this._ongoingRequest = null;
        res(e);
      });
    });
    return this._ongoingRequest;
  }

  private _checkResponse(request: Request, resp: Response): void {
    if (request.request !== resp.response) {
      console.warn('received response for wrong request');
      throw new Error('received response for wrong request');
    }
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

export class TrajectorySegmentManager {
  static getSegmentsById(segments: TrajectoryResponse, id: number): RawKnot[] | undefined {
    const trajectory = segments.values.find(element => element.id === id);
    return trajectory?.segments;
  }

  static getPositionXY(segment: RawKnot): TrajectoryCoords {
    return { x: segment.x[0], y: segment.x[1] };
  }
  /**
   * Checks for duplicates on a list of type TrajectoryCoords[]. Returns the list of duplicated elements.
   * @param positions list of element to to check for duplicates
   */
  static getRepeatedCoords(positions: TrajectoryCoords[]): TrajectoryCoords[] {
    // Get duplicated coords
    const seenCoords = positions.filter(
      (set => (position: any) =>
        set.has(JSON.stringify(position)) || !set.add(JSON.stringify(position)))(new Set()),
    );
    // Return a unique list of duplicated coords.
    let uniqueCoords: TrajectoryCoords[] = [];
    for (let index = 0; index < seenCoords.length; index++) {
      const element = seenCoords[index];
      if (!uniqueCoords.some(e => JSON.stringify(e) === JSON.stringify(element))) {
        uniqueCoords.push(element);
      }
    }
    return uniqueCoords;
  }

  static getConflictsInSetOfSegments(segments: RawKnot[], conflictPoints: TrajectoryCoords[]) {
    let conflictSegments: RawKnot[] = []
    for (let index = 0; index < segments.length; index++) {
      const segment = segments[index];
      for (let index = 0; index < conflictPoints.length; index++) {
        const conflictPoint = conflictPoints[index];
        if (segment.x[0] === conflictPoint.x && segment.x[1] === conflictPoint.y) {
          conflictSegments.push(segment);
        }
      }
    }
    return conflictSegments;
  }
  static getConflictSegments(trajectoriesSegments: RawKnot[][], conflictPoints: TrajectoryCoords[]) {
    let conflictSegments: RawKnot[][] = []
    trajectoriesSegments.forEach(segments => {
      conflictSegments.push(TrajectorySegmentManager.getConflictsInSetOfSegments(segments, conflictPoints));
    });
    return conflictSegments;
  }

}

type WebSocketSendParam0T = Parameters<WebSocket['send']>[0];
