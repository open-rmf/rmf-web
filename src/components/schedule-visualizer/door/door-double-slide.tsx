import React from 'react';
import { DoorProps } from './door';
import SingleSlideDoor from './door-single-slide';

/*
 * Double sliding doors:
 *  - door panels slide from the centerpoint of v1<->v2 towards v1 and v2
 *  - the door slides from (v2_x, v2_y) towards (v1_x, v1_y)
 */
const DoubleSlideDoor = function(props: DoorProps): React.ReactElement {
  const { door, v1, v2, onClick, currentMode } = props;
  const [v1_x, v1_y] = v1;
  const [v2_x, v2_y] = v2;
  const [hingeX1, hingeY1, hingeX2, hingeY2] = [v1_x, v1_y, v2_x, v2_y];
  const [extendX1, extendY1] = [hingeX1 + (v2_x - v1_x) / 2, hingeY1 + (v2_y - v1_y) / 2];

  return (
    <>
      <SingleSlideDoor
        v1={[hingeX1, -hingeY1]}
        v2={[extendX1, -extendY1]}
        door={door}
        onClick={onClick}
        currentMode={currentMode}
      />
      <SingleSlideDoor
        v1={[extendX1, -extendY1]}
        v2={[hingeX2, -hingeY2]}
        door={door}
        onClick={onClick}
        currentMode={currentMode}
      />
    </>
  );
};

export default DoubleSlideDoor;
