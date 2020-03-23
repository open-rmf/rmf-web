import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import ColorManager from './colors';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const useStyles = makeStyles(() => ({
  robotMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));

export interface RobotsOverlayProps extends SVGOverlayProps {
  robots: readonly RomiCore.RobotState[];
  colorManager: ColorManager;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

export default function RobotsOverlay(props: RobotsOverlayProps): React.ReactElement {
  const classes = useStyles();
  const { robots, colorManager, onRobotClick, ...otherProps } = props;

  const [pendingColors, setPendingColors] = React.useState<RomiCore.RobotState[]>([]);

  const bounds =
    props.bounds instanceof L.LatLngBounds ? props.bounds : new L.LatLngBounds(props.bounds);
  const width = bounds.getEast() - bounds.getWest();
  const height = bounds.getNorth() - bounds.getSouth();
  const viewBox = `0 0 ${width} ${height}`;

  React.useEffect(() => {
    if (!pendingColors.length) {
      return;
    }
    (async () => {
      await Promise.all(
        pendingColors.map(async robot => colorManager.robotColor(robot.name, robot.model)),
      );
      setPendingColors([]);
    })();
  });

  function handleRobotClick(robot: RomiCore.RobotState): void {
    onRobotClick && onRobotClick(robot);
  }

  const footprint = 0.5; // hardcode for now, footprint data not available.
  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {robots.map(robot => {
          const robotColor = colorManager.robotColorFromCache(robot.name, robot.model);
          if (!robotColor) {
            pendingColors.push(robot);
            return null;
          }
          const nose = [
            robot.location.x + Math.cos(robot.location.yaw) * footprint,
            robot.location.y + Math.sin(robot.location.yaw) * footprint,
          ];
          return (
            <g key={robot.name}>
              <filter id={`${robot.name}-shadow`} x="-20%" y="-20%" width="140%" height="140%">
                <feDropShadow dx="0" dy="0" stdDeviation={footprint * 0.15} floodColor="black" />
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
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
