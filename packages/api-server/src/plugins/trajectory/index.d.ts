import WebSocket from 'ws';
import { Argv } from 'yargs';
import ApiGateway from '../../api-gateway';
export declare function options(
  yargs: Argv,
): Argv<{
  trajectoryServerUrl: string;
}>;
declare type ConfigType = ReturnType<typeof options>['argv'];
export declare function onLoad(config: ConfigType, api: ApiGateway): Promise<void>;
interface TrajectoryRequest {
  request: 'trajectory';
  mapName: string;
  duration: number;
  trim: boolean;
}
export declare type LatestTrajectoryParams = Omit<TrajectoryRequest, 'request'>;
export declare type RawVelocity = [number, number, number];
export declare type RawPose2D = [number, number, number];
export interface RawKnot {
  t: number;
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
export declare type Conflict = number[];
interface TrajectoryResponse {
  response: 'trajectory';
  values: Trajectory[];
  conflicts: Conflict[];
}
export declare type LatestTrajectoryResult = Omit<TrajectoryResponse, 'response'>;
interface TimeRequest {
  request: 'time';
  param: {};
}
interface TimeResponse {
  response: 'time';
  values: [number];
}
export declare type TrajectoryServerTimeParams = Omit<TimeRequest, 'request'>;
export declare type TrajectoryServerTimeResult = Omit<TimeResponse, 'response'>;
export default class TrajectoryPlugin {
  private _config;
  socket?: WebSocket;
  constructor(_config: ConfigType);
  latestTrajectory(params: LatestTrajectoryParams): Promise<LatestTrajectoryResult>;
  serverTime(params: TrajectoryServerTimeParams): Promise<TrajectoryServerTimeResult>;
  private static _connect;
  private _ongoingRequests;
  private _send;
}
export {};
