import React from 'react';
import doorStyles from './door-style';
import { DoorProps, getDoorStyle } from './door';

/**
 * Double hinge doors:
 * - hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
 * - motion range = door swing ranges in DEGREES (assume symmetric)
 * - same motion-direction selection as single hinge
 */
const DoubleHingeDoor = React.forwardRef(function(
  props: DoorProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { door, onClick, currentMode } = props;
  const { v1_x, v1_y, v2_x, v2_y } = door;
  const [hingeX1, hingeY1, hingeX2, hingeY2] = [v1_x, v1_y, v2_x, v2_y];
  const [extendX1, extendY1] = [hingeX1 + (v2_x - v1_x) / 2, hingeY1 + (v2_y - v1_y) / 2];
  const classes = doorStyles();
  const doorStyle = getDoorStyle(classes, currentMode);
  return (
    <>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={`${classes.doorMarker} ${classes.door} ${doorStyle}`}
          x1={hingeX1}
          y1={-hingeY1}
          x2={extendX1}
          y2={-extendY1}
        />
      </g>
      <g ref={ref} onClick={e => onClick && onClick(e as any, door)}>
        <line
          className={`${classes.doorMarker} ${classes.door} ${doorStyle}`}
          x1={extendX1}
          y1={-extendY1}
          x2={hingeX2}
          y2={-hingeY2}
        />
      </g>
    </>
  );
});

export default DoubleHingeDoor;
