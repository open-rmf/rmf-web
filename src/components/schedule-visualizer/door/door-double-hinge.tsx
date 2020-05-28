import React from 'react';
import { DoorProps } from './door';
import SingleHingeDoor from './door-single-hinge';

/**
 * Double hinge doors:
 * - hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
 * - motion range = door swing ranges in DEGREES (assume symmetric)
 * - same motion-direction selection as single hinge
 */
const DoubleHingeDoor = function(props: DoorProps): React.ReactElement {
  const { door, v1, v2, onClick, currentMode } = props;
  const [v1_x, v1_y] = v1;
  const [v2_x, v2_y] = v2;
  const [hingeX1, hingeY1, hingeX2, hingeY2] = [v1_x, v1_y, v2_x, v2_y];
  const [extendX1, extendY1] = [hingeX1 + (v2_x - v1_x) / 2, hingeY1 + (v2_y - v1_y) / 2];
  return (
    <>
      <SingleHingeDoor
        v1={[hingeX1, hingeY1]}
        v2={[extendX1, extendY1]}
        door={door}
        onClick={onClick}
        currentMode={currentMode}
      />
      <SingleHingeDoor
        v1={[extendX1, extendY1]}
        v2={[hingeX2, hingeY2]}
        door={door}
        onClick={onClick}
        currentMode={currentMode}
      />
    </>
  );
};

export default DoubleHingeDoor;
