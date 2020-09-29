import * as assert from 'assert';
import WebSocket from 'ws';
import { Argv } from 'yargs';
import ApiGateway from '../../api-gateway';
import logger from '../../logger';

export function options(yargs: Argv) {
  return yargs.option('trajectoryServerUrl', {
    type: 'string',
    demandOption: true,
  });
}

type ConfigType = ReturnType<typeof options>['argv'];

export async function onLoad(config: ConfigType, api: ApiGateway): Promise<void> {
  const plugin = new TrajectoryPlugin(config);
  api.registerHandler('latestTrajectory', (params) => plugin.latestTrajectory(params));
  api.registerHandler('trajectoryServerTime', (params) => plugin.serverTime(params));
}

interface TrajectoryRequest {
  request: 'trajectory';
  mapName: string;
  duration: number;
  trim: boolean;
}

export type LatestTrajectoryParams = Omit<TrajectoryRequest, 'request'>;

// RawVelocity received from server is in this format (x, y, theta)
export type RawVelocity = [number, number, number];

// RawPose2D received from server is in this format (x, y, theta)
export type RawPose2D = [number, number, number];

export interface RawKnot {
  t: number; // milliseconds
  v: RawVelocity;
  x: RawPose2D;
}

export interface Trajectory {
  id: number;
  shape: string;
  dimensions: number;
  segments: RawKnot[];
  robot_name: string;
  fleet_name: string;
}

export type Conflict = number[];

interface TrajectoryResponse {
  response: 'trajectory';
  values: Trajectory[];
  conflicts: Conflict[];
}

export type LatestTrajectoryResult = Omit<TrajectoryResponse, 'response'>;

interface TimeRequest {
  request: 'time';
  param: {};
}

interface TimeResponse {
  response: 'time';
  values: [number];
}

export type TrajectoryServerTimeParams = Omit<TimeRequest, 'request'>;
export type TrajectoryServerTimeResult = Omit<TimeResponse, 'response'>;

export default class TrajectoryPlugin {
  socket?: WebSocket;

  constructor(private _config: ConfigType) {
  }

  async latestTrajectory(params: LatestTrajectoryParams): Promise<LatestTrajectoryResult> {
    const request: TrajectoryRequest = {
      request: 'trajectory',
      ...params,
    };
    this._send(JSON.stringify(request));

    const data = await new Promise((res) => this._ongoingRequests.push(res));
    assert.ok(typeof data === 'string');
    const resp = JSON.parse(data) as TrajectoryResponse;
    assert.strictEqual(resp.response, 'trajectory');

    if (resp.values === null) {
      resp.values = [];
    }
    delete (resp as any).response;
    return resp as LatestTrajectoryResult;
  }

  async serverTime(params: TrajectoryServerTimeParams): Promise<TrajectoryServerTimeResult> {
    const request: TimeRequest = {
      request: 'time',
      ...params,
    };
    this._send(JSON.stringify(request));

    const data = await new Promise((res) => this._ongoingRequests.push(res));
    assert.ok(typeof data === 'string');
    const resp = JSON.parse(data) as TimeResponse;
    assert.strictEqual(resp.response, 'time');

    delete (resp as any).response;
    return resp as TrajectoryServerTimeResult;
  }

  private static async _connect(config: ConfigType): Promise<WebSocket> {
    logger.info(`connecting to trajectory server at ${config.trajectoryServerUrl}`);
    const socket = new WebSocket(config.trajectoryServerUrl, { handshakeTimeout: 5000 });

    await new Promise((res, rej) => {
      const errorHandler = (error: Error) => rej(error);
      socket.on('error', errorHandler);
      socket.once('open', () => {
        socket.off('close', errorHandler);
        res();
      });
    });
    logger.info('succesfully connected to trajectory server');
    return socket;
  }

  private _ongoingRequests: ((data: unknown) => void)[] = [];

  private async _send(payload: WebSocket.Data): Promise<void> {
    if (!this.socket) {
      this._ongoingRequests = [];
      this.socket = await TrajectoryPlugin._connect(this._config);
      this.socket.on('message', (data) => {
        const resolve = this._ongoingRequests.shift();
        resolve && resolve(data);
      });
    }
    this.socket.send(payload);
  }
}
