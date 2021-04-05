import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { DoorState, FleetState, io, LiftState, SioClient } from 'api-client';
import React from 'react';
import appConfig from '../../app-config';
import DispenserStateManager from '../../managers/dispenser-state-manager';
import FleetManager from '../../managers/fleet-manager';
import LiftStateManager from '../../managers/lift-state-manager';
import {
  NegotiationConflict,
  NegotiationStatusManager,
} from '../../managers/negotiation-status-manager';
import { HealthStatus } from '../../managers/rmf-health-state-manager';
import { RobotTrajectoryManager } from '../../managers/robot-trajectory-manager';
import TaskManager from '../../managers/task-manager';

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

export const DispenserStateContext = React.createContext<Record<string, RomiCore.DispenserState>>(
  {},
);
export const DoorStateContext = React.createContext<Record<string, DoorState>>({});
export const FleetStateContext = React.createContext<Record<string, FleetState>>({});
export const LiftStateContext = React.createContext<Record<string, LiftState>>({});
export const NegotiationStatusContext = React.createContext<Record<number, NegotiationConflict>>(
  {},
);
export const TasksContext = React.createContext<RomiCore.TaskSummary[]>([]);
export const TransportContext = React.createContext<RomiCore.Transport | null>(null);
export const BuildingMapContext = React.createContext<RomiCore.BuildingMap | null>(null);
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
  dispenserStateManager: DispenserStateManager;
  fleetManager: FleetManager;
  liftStateManager: LiftStateManager;
  negotiationStatusManager: NegotiationStatusManager;
  taskManager: TaskManager;
  trajectoryManager?: RobotTrajectoryManager;
}

export const RmfIngressContext = React.createContext<RmfIngress>({
  sioClient,
  dispenserStateManager: new DispenserStateManager(),
  fleetManager: new FleetManager(),
  liftStateManager: new LiftStateManager(),
  negotiationStatusManager: new NegotiationStatusManager(appConfig.trajServerUrl),
  taskManager: new TaskManager(),
});
