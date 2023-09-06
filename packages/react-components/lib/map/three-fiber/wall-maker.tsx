import React from 'react';
import { Graph } from 'api-client';
import { CubeMaker } from './cube-maker';
import { graphToWalls } from '../utils';

interface WallProps {
  wallGraph: Graph;
}

export const WallMaker = ({ wallGraph }: WallProps): JSX.Element => {
  const walls = graphToWalls(wallGraph);

  return (
    <>
      {walls.map((wall, i) => {
        return (
          <CubeMaker
            key={i}
            rot={wall.rot}
            size={[wall.width, wall.depth, wall.height]}
            position={[wall.position[0], wall.position[1], wall.position[2]]}
          />
        );
      })}
    </>
  );
};
