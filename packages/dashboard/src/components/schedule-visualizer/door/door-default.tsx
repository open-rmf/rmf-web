import React from 'react';
import doorStyles from './door-style';

export interface defaultDoorProps {
  v1: number[];
  v2: number[];
  style?: string;
  onClick?(e: React.MouseEvent<SVGGElement>): void;
}

const DefaultDoor = function(props: defaultDoorProps): React.ReactElement {
  const { v1, v2, style, onClick } = props;
  const [v1_x, v1_y] = v1;
  const [v2_x, v2_y] = v2;
  const classes = doorStyles();

  return (
    <g onClick={event => onClick && onClick(event)}>
      <line
        className={`${classes.doorMarker} ${classes.door} ${style ? style : classes.undefinedDoor}`}
        x1={v1_x}
        y1={v1_y}
        x2={v2_x}
        y2={v2_y}
      />
    </g>
  );
};

export default DefaultDoor;
