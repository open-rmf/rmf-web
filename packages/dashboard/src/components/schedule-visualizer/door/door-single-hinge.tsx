import React from 'react';
import doorStyles from './door-style';
import { DoorProps, getDoorStyle } from './door';
import DefaultDoor from './door-default';

/*
 * Single hinge doors:
 *  - hinge is located at (v1_x, v1_y)
 *  - door extends till (v2_x, v2_y)
 *  - motion_range = door swing range in DEGREES
 *  - there are two possible motions: clockwise and anti-clockwise
 *  - selected by the motion_direction parameter, which is +1 or -1
 */
const SingleHingeDoor = function(props: DoorProps): React.ReactElement {
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

export default SingleHingeDoor;
