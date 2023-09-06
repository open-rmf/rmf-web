import React from 'react';
import { Level } from 'api-client';
import { Wall } from './wall-maker';

export interface BuildingMapProps {
  level: Level;
}

export const BuildingCubes = ({ level }: BuildingMapProps): JSX.Element => {
  return <>{level.wall_graph.edges.length > 0 ? <Wall wallGraph={level.wall_graph} /> : null}</>;
};
