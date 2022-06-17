import { Dispenser, FleetState, Ingestor } from 'api-client';
import React from 'react';
import { RmfIngress } from './rmf-ingress';

export const FleetsContext = React.createContext<FleetState[]>([]);
export const DispensersContext = React.createContext<Dispenser[]>([]);
export const IngestorsContext = React.createContext<Ingestor[]>([]);

export const RmfIngressContext = React.createContext<RmfIngress | undefined>(undefined);
