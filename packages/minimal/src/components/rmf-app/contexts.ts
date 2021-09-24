import React from 'react';
import * as RmfModels from 'rmf-models';
import { Place } from 'react-components';

export const BuildingMapContext = React.createContext<RmfModels.BuildingMap | null>(null);
export const PlacesContext = React.createContext<Place[]>([]);
