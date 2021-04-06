import { Configuration, DoorsApi, io, LiftsApi, SioClient, TasksApi } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import appConfig from '../../app-config';
import {
  NegotiationConflict,
  NegotiationStatusManager,
} from '../../managers/negotiation-status-manager';
import { HealthStatus } from '../../managers/rmf-health-state-manager';
import { RobotTrajectoryManager } from '../../managers/robot-trajectory-manager';

const itemSummary = () => {
  return {
    operational: 0,
    spoiltItem: [],
  };
};

const initializeItemSummary = itemSummary();

const sioClient = (() => {
  const token = appConfig.authenticator.token;
  const options: Parameters<typeof io>[1] = {};
  if (token) {
    options.auth = { token };
  }
  if (process.env.REACT_APP_RMF_SERVER) {
    return io(process.env.REACT_APP_RMF_SERVER, options);
  } else {
    return io(options);
  }
})();
sioClient.on('error', console.error);

export const DispenserStateContext = React.createContext<Record<string, RmfModels.DispenserState>>(
  {},
);
export const DoorStateContext = React.createContext<Record<string, RmfModels.DoorState>>({});
export const FleetStateContext = React.createContext<Record<string, RmfModels.FleetState>>({});
export const LiftStateContext = React.createContext<Record<string, RmfModels.LiftState>>({});
export const NegotiationStatusContext = React.createContext<Record<number, NegotiationConflict>>(
  {},
);
export const TasksContext = React.createContext<Record<string, RmfModels.TaskSummary>>({});
export const BuildingMapContext = React.createContext<RmfModels.BuildingMap | null>(null);
export const RmfHealthContext = React.createContext<HealthStatus>({
  door: { ...initializeItemSummary },
  lift: { ...initializeItemSummary },
  dispenser: { ...initializeItemSummary },
  robot: {
    operational: 0,
    idle: 0,
    charging: 0,
    spoiltRobots: [],
  },
});

export interface RmfIngress {
  sioClient: SioClient;
  doorsApi: DoorsApi;
  liftsApi: LiftsApi;
  tasksApi: TasksApi;
  negotiationStatusManager: NegotiationStatusManager;
  trajectoryManager?: RobotTrajectoryManager;
}

const apiConfig: Configuration = {
  accessToken: appConfig.authenticator.token,
  basePath: appConfig.rmfServerUrl,
};
export const RmfIngressContext = React.createContext<RmfIngress>({
  sioClient,
  doorsApi: new DoorsApi(apiConfig),
  liftsApi: new LiftsApi(),
  tasksApi: new TasksApi(),
  negotiationStatusManager: new NegotiationStatusManager(appConfig.trajServerUrl),
});
