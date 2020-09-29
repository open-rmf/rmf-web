import React from 'react';
import doorStyles from './door-style';
import { DoorProps, getDoorStyle } from './door';
import DefaultDoor from './door-default';

/*
 * Single sliding doors:
 *  - the door slides from (v2_x, v2_y) towards (v1_x, v1_y)
 *  - range of motion is entire distance from v2->v1. No need to specify.
 */
const SingleSlideDoor = function(props: DoorProps): React.ReactElement {
  const { currentMode, onClick, v1, v2 } = props;
  const [v1_x, v1_y] = v1;
  const [v2_x, v2_y] = v2;
  const [hingeX, hingeY, extendX, extendY] = [v1_x, v1_y, v2_x, v2_y];
  const classes = doorStyles();

  return (
    <>
      <DefaultDoor
        v1={[hingeX, -hingeY]}
        v2={[extendX, -extendY]}
        style={getDoorStyle(classes, currentMode)}
      />
      <DefaultDoor
        v1={[hingeX, -hingeY]}
        v2={[extendX, -extendY]}
        style={classes.doorTransparent}
        onClick={onClick}
      />
    </>
  );
};

export default SingleSlideDoor;
