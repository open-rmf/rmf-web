import React from 'react';
import { useTheme } from '@material-ui/core';
import { Trajectory } from '../../robot-trajectory-manager';

export interface TrajectoryConflictProps {
  pathD: string;
  trajectory: Trajectory;
  trajectoryDiameter: number;
}

export default function RobotTrajectoryConflict(props: TrajectoryConflictProps) {
  const { pathD, trajectory, trajectoryDiameter } = props;
  const theme = useTheme();

  return (
    <React.Fragment>
      {/* <filter
        id={`shadow`}
        x="-20%"
        y="-20%"
        width="140%"
        height="140%"
        filterUnits="userSpaceOnUse"
        >
        <feColorMatrix
          type="matrix"
          values="1 0 0 0 0
                                      0 1 0 0 0
                                      0 0 1 0 0
                                      0 0 0 1 0"
          result="boostedInput"
        />
  
        <feGaussianBlur stdDeviation={trajectoryDiameter * 0.4} />
        <feComposite operator="out" in2="boostedInput" />
      </filter> */}
      <mask width="100" id={`${trajectory.id}-mask`}>
        <rect x={0} y={0} width={'100%'} height={'100%'} fill={'white'} />
        <path
          id="maskPath"
          d={pathD}
          stroke={'black'}
          strokeWidth={trajectoryDiameter - 0.1}
          strokeLinecap="round"
          fill={'none'}
          strokeDasharray={2}
          strokeDashoffset={0}
          pathLength={1}
          opacity={1}
        />
      </mask>
      <path
        id="errorPath"
        d={pathD}
        stroke={theme.palette.secondary.main}
        strokeWidth={trajectoryDiameter}
        strokeLinecap="round"
        fill={'none'}
        // filter={`url(#shadow)`}
        strokeDasharray={2}
        strokeDashoffset={0}
        pathLength={1}
        opacity={1}
        mask={`url(#${trajectory.id}-mask)`}
      />
    </React.Fragment>
  );
}
