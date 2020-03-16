import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { computeRobotColor } from './colors';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const useStyles = makeStyles(() => ({
  robotMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));

export interface RobotsOverlayProps extends SVGOverlayProps {
  fleets: readonly RomiCore.FleetState[];
  onRobotClick?(robot: RomiCore.RobotState): void;
}

function robotColorKey(robot: RomiCore.RobotState): string {
  return `${robot.model}__${robot.name}`;
}

export default function RobotsOverlay(props: RobotsOverlayProps): React.ReactElement {
  const classes = useStyles();

  const [robotColors, setRobotColors] = React.useState<Record<string, string>>({});

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  React.useEffect(() => {
    setRobotColors(robotColors => {
      (async () => {
        let updated = false;
        for (const robot of props.fleets.flatMap(f => f.robots)) {
          const key = robotColorKey(robot);
          if (robotColors[key] === undefined) {
            robotColors[key] = await computeRobotColor(robot.name, robot.model);
            updated = true;
          }
        }
        if (updated) {
          setRobotColors({ ...robotColors });
        }
      })();
      return robotColors;
    });
  }, [props.fleets]);

  function handleRobotClick(robot: RomiCore.RobotState): void {
    props.onRobotClick && props.onRobotClick(robot);
  }

  const footprint = 0.5; // hardcode for now, footprint data not available.
  return (
    <SVGOverlay {...props}>
      <svg viewBox={viewBox}>
        {props.fleets
          .flatMap(x => x.robots)
          .map(robot => {
            const robotColor = robotColors[robotColorKey(robot)];
            const nose = [
              robot.location.x + Math.cos(robot.location.yaw) * footprint,
              robot.location.y + Math.sin(robot.location.yaw) * footprint,
            ];
            return (
              robotColor && (
                <g key={robot.name}>
                  <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
                    <feDropShadow
                      dx="0"
                      dy="0"
                      stdDeviation={footprint * 0.15}
                      floodColor="black"
                    />
                  </filter>
                  <circle
                    className={classes.robotMarker}
                    onClick={() => handleRobotClick(robot)}
                    cx={robot.location.x}
                    cy={-robot.location.y}
                    r={footprint}
                    fill={robotColor}
                    filter={`url(#${robot.name}-shadow)`}
                  />
                  <line
                    x1={robot.location.x}
                    y1={-robot.location.y}
                    x2={nose[0]}
                    y2={-nose[1]}
                    stroke="black"
                    strokeWidth="0.05"
                  />
                </g>
              )
            );
          })}
      </svg>
    </SVGOverlay>
  );
}
