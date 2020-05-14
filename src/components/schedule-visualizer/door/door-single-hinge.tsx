import React from 'react';
import doorStyles from './door-style';
import { DoorProps, getDoorStyle } from './door';

/*
 * Single hinge doors:
 *  - hinge is located at (v1_x, v1_y)
 *  - door extends till (v2_x, v2_y)
 *  - motion_range = door swing range in DEGREES
 *  - there are two possible motions: clockwise and anti-clockwise
 *  - selected by the motion_direction parameter, which is +1 or -1
 */
const SingleHingeDoor = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const [hingeX, hingeY, extendX, extendY] = [v1_x, v1_y, v2_x, v2_y];
  const classes = doorStyles();

  return (
    <>
      <g>
        <line
          className={`${classes.doorMarker} ${classes.door} ${getDoorStyle(classes, currentMode)}`}
          x1={hingeX}
          y1={-hingeY}
          x2={extendX}
          y2={-extendY}
        />
      </g>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={`${classes.doorMarker} ${classes.door} ${classes.doorTransparent}`}
          x1={hingeX}
          y1={-hingeY}
          x2={extendX}
          y2={-extendY}
        />
      </g>
    </>
  );
});

export default SingleHingeDoor;
