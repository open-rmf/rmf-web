import { Dispenser, Fleet, Ingestor } from 'api-client';
import React from 'react';
import { Place } from 'react-components';
import * as RmfModels from 'rmf-models';
import { RmfIngress } from './rmf-ingress';

export const BuildingMapContext = React.createContext<RmfModels.BuildingMap | null>(null);
export const PlacesContext = React.createContext<Place[]>([]);
export const FleetsContext = React.createContext<Fleet[]>([]);
export const DispensersContext = React.createContext<Dispenser[]>([]);
export const IngestorsContext = React.createContext<Ingestor[]>([]);

export const RmfIngressContext = React.createContext<RmfIngress | undefined>(undefined);
