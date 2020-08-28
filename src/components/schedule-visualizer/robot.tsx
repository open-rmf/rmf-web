import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React, { useContext, useState } from 'react';
import { ResourcesContext } from '../../app-contexts';
import ResourceManager from '../../resource-manager';
import ColorManager from './colors';
import RobotDefaultIcon from './robot-default-icon';
import RobotImageIcon from './robot-image-icon';

const useStyles = makeStyles(() => ({
  robotText: {
    dominantBaseline: 'central',
    textAnchor: 'middle',
    fontSize: '0.18px',
    fontWeight: 'bold',
    fill: 'white',
    /* 1 pixel black shadow to left, top, right and bottom */
    textShadow: '-1px 0 black, 0 1px black, 1px 0 black, 0 -1px black',
    pointerEvents: 'none',
  },

  container: {
    pointerEvents: 'visible',
    cursor: 'pointer',
  },
}));

export interface RobotProps {
  robot: RomiCore.RobotState;
  colorManager: ColorManager;
  footprint: number;
  fleetName: string;
  inConflict?: boolean;
  onClick?(e: React.MouseEvent<SVGGElement>, robot: RomiCore.RobotState): void;
}

const Robot = React.forwardRef(function(
  props: RobotProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const resourcesContext = useContext(ResourcesContext);
  const classes = useStyles();
  const { robot, footprint, colorManager, fleetName, inConflict, onClick } = props;
  // The only image formats SVG software support are JPEG, PNG, and other SVG files.
  const [renderCustomIcon, setRenderCustomIcon] = useState({
    path: ResourceManager.getRobotIconPath(resourcesContext, fleetName),
    error: false,
  });
  const textRef = React.useRef<SVGTextElement>(null);

  React.useEffect(() => {
    const text = textRef.current;
    if (!text) {
      return;
    }
    // svg text does not support auto ellipses, this workaround by testing the text length and
    // truncate it bit by bit until it fits the icon. Its abit hacky but it shouldn't be too bad
    // unless the robot name is mega long.
    for (text.textContent = robot.name; text.getComputedTextLength() > footprint * 1.9; ) {
      text.textContent = text.textContent!.slice(0, text.textContent!.length - 6) + '...';
    }
    console.log(textRef.current?.textContent, textRef.current?.getComputedTextLength());
  }, [robot.name, footprint]);

  return (
    <>
      <g
        ref={ref}
        data-component="Robot"
        className={`${classes.container}`}
        aria-label={robot.name}
        onClick={e => onClick && onClick(e, robot)}
        transform={`translate(${robot.location.x} ${-robot.location.y})
            rotate(${-(robot.location.yaw * 180) / Math.PI})`}
      >
        {!!renderCustomIcon.path && !renderCustomIcon.error ? (
          <RobotImageIcon
            iconPath={renderCustomIcon.path}
            robot={robot}
            footprint={footprint}
            dispatchIconError={setRenderCustomIcon}
            inConflict={inConflict}
          />
        ) : (
          <RobotDefaultIcon
            robot={robot}
            footprint={footprint}
            colorManager={colorManager}
            inConflict={inConflict}
          ></RobotDefaultIcon>
        )}
      </g>
      <text
        ref={textRef}
        id="robotName"
        x={robot.location.x}
        y={-robot.location.y}
        className={classes.robotText}
      />
    </>
  );
});

export default Robot;
