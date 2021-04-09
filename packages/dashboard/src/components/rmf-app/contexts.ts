import React from 'react';
import * as RmfModels from 'rmf-models';
import { NegotiationConflict } from '../../managers/negotiation-status-manager';
import { HealthStatus } from '../../managers/rmf-health-state-manager';
import { RmfIngress } from './rmf-ingress';

const itemSummary = () => {
  return {
    operational: 0,
    spoiltItem: [],
  };
};

const initializeItemSummary = itemSummary();

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

export const RmfIngressContext = React.createContext(new RmfIngress());
