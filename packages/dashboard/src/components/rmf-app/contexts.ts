import { BuildingMap, Dispenser, FleetState, Ingestor } from 'api-client';
import React from 'react';
import { Place } from 'react-components';
import { RmfIngress } from './rmf-ingress';

export const BuildingMapContext = React.createContext<BuildingMap | null>(null);
export const PlacesContext = React.createContext<Place[]>([]);
export const FleetsContext = React.createContext<FleetState[]>([]);
export const DispensersContext = React.createContext<Dispenser[]>([]);
export const IngestorsContext = React.createContext<Ingestor[]>([]);

export const RmfIngressContext = React.createContext<RmfIngress | undefined>(undefined);
