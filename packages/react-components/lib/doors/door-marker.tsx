import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { joinClasses } from '../css-utils';

const debug = Debug('Doors:DoorMarker');

const useDoorStyles = makeStyles({
  marker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  base: {
    strokeWidth: 0.2,
  },
  open: {
    stroke: '#AFDDAE',
    strokeDasharray: 0.1,
  },
  close: {
    stroke: '#BC4812',
  },
  moving: {
    stroke: '#E9CE9F',
    strokeDasharray: 0.3,
  },
  unknown: {
    stroke: 'grey',
  },
  transparent: {
    stroke: 'transparent',
  },
});

function useDoorStyle(doorState?: RomiCore.DoorState): string {
  const classes = useDoorStyles();

  if (!doorState) {
    return classes.unknown;
  }

  switch (doorState.current_mode.value) {
    case RomiCore.DoorMode.MODE_OPEN:
      return classes.open;
    case RomiCore.DoorMode.MODE_MOVING:
      return classes.moving;
    case RomiCore.DoorMode.MODE_CLOSED:
      return classes.close;
    default:
      return classes.unknown;
  }
}

interface BaseDoorProps {
  v1: [number, number];
  v2: [number, number];
  className?: string;
}

const BaseDoor = (props: BaseDoorProps) => {
  const { v1, v2, className } = props;
  const classes = useDoorStyles();

  return (
    <g>
      <line
        className={joinClasses(classes.base, className)}
        x1={v1[0]}
        y1={v1[1]}
        x2={v2[0]}
        y2={v2[1]}
      />
    </g>
  );
};

interface DummyDoorProps {
  v1: [number, number];
  v2: [number, number];
}

/**
 * Because we are using stroke-dash in some of the classes, it makes it such that only
 * the rendered line will be considered for click detection. To workaround it, we use
 * a transparent door on top of the marker, this dummy door will be used to allow the
 * full door to be clickable.
 */
const DummyDoor = (props: DummyDoorProps) => {
  const { v1, v2 } = props;
  const classes = useDoorStyles();

  return (
    <g>
      <line
        className={joinClasses(classes.base, classes.transparent)}
        x1={v1[0]}
        y1={v1[1]}
        x2={v2[0]}
        y2={v2[1]}
      />
    </g>
  );
};

type DoorMarkerImplProps = Omit<DoorMarkerProps, 'onClick'>;

/*
 * Single swing doors:
 *  - hinge is located at (v1_x, v1_y)
 *  - door extends till (v2_x, v2_y)
 *  - motion_range = door swing range in DEGREES
 *  - there are two possible motions: clockwise and anti-clockwise
 *  - selected by the motion_direction parameter, which is +1 or -1
 */
const SingleSwingDoor = (props: DoorMarkerImplProps) => {
  const { door, doorState } = props;
  const doorStyle = useDoorStyle(doorState);

  return (
    <>
      <BaseDoor v1={[door.v1_x, door.v1_y]} v2={[door.v2_x, door.v2_y]} className={doorStyle} />
      <DummyDoor v1={[door.v1_x, door.v1_y]} v2={[door.v2_x, door.v2_y]} />
    </>
  );
};

/*
 * Single sliding doors:
 *  - the door slides from (v2_x, v2_y) towards (v1_x, v1_y)
 *  - range of motion is entire distance from v2->v1. No need to specify.
 */
const SingleSlidingDoor = SingleSwingDoor;

/**
 * Double hinge doors:
 * - hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
 * - motion range = door swing ranges in DEGREES (assume symmetric)
 * - same motion-direction selection as single hinge
 */
const DoubleSwingDoor = (props: DoorMarkerImplProps) => {
  const { door, doorState } = props;
  const [hingeX1, hingeY1, hingeX2, hingeY2] = [door.v1_x, door.v1_y, door.v2_x, door.v2_y];
  const [extendX1, extendY1] = [
    hingeX1 + (door.v2_x - door.v1_x) / 2,
    hingeY1 + (door.v2_y - door.v1_y) / 2,
  ];
  const doorStyle = useDoorStyle(doorState);
  return (
    <>
      <BaseDoor v1={[hingeX1, hingeY1]} v2={[extendX1, extendY1]} className={doorStyle} />
      <BaseDoor v1={[extendX1, extendY1]} v2={[hingeX2, hingeY2]} className={doorStyle} />
      <DummyDoor v1={[hingeX1, hingeY1]} v2={[extendX1, extendY1]} />
    </>
  );
};

/*
 * Double sliding doors:
 *  - door panels slide from the centerpoint of v1<->v2 towards v1 and v2
 *  - the door slides from (v2_x, v2_y) towards (v1_x, v1_y)
 */
const DoubleSlidingDoor = DoubleSwingDoor;

/**
 * door: Door information provided by the map.
 * doorState: Current state of the door.
 * onClick: Action to trigger on click.
 */
export interface DoorMarkerProps extends React.SVGProps<SVGGElement> {
  door: RomiCore.Door;
  doorState?: RomiCore.DoorState;
}

export const DoorMarker = React.memo(
  React.forwardRef((props: DoorMarkerProps, ref: React.Ref<SVGGElement>) => {
    const { door, doorState, className, ...otherProps } = props;
    debug(`render ${door.name}`);
    const classes = useDoorStyles();
    console.log(otherProps.onClick);

    const renderDoor = () => {
      switch (door.door_type) {
        case RomiCore.Door.DOOR_TYPE_SINGLE_SWING:
          return <SingleSwingDoor door={door} doorState={doorState} />;
        case RomiCore.Door.DOOR_TYPE_SINGLE_SLIDING:
          return <SingleSlidingDoor door={door} doorState={doorState} />;
        case RomiCore.Door.DOOR_TYPE_DOUBLE_SWING:
          return <DoubleSwingDoor door={door} doorState={doorState} />;
        case RomiCore.Door.DOOR_TYPE_DOUBLE_SLIDING:
          return <DoubleSlidingDoor door={door} doorState={doorState} />;
        default:
          return null;
      }
    };

    return (
      <g ref={ref} className={joinClasses(classes.marker, className)} {...otherProps}>
        {renderDoor()}
      </g>
    );
  }),
);

export default DoorMarker;
