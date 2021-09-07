import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import Debug from 'debug';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { fromRmfCoords } from '../utils/geometry';

const debug = Debug('Map:DoorMarker');

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

function useDoorStyle(doorMode?: number): string {
  const classes = useDoorStyles();

  if (doorMode === undefined) {
    return classes.unknown;
  }

  switch (doorMode) {
    case RmfModels.DoorMode.MODE_OPEN:
      return classes.open;
    case RmfModels.DoorMode.MODE_MOVING:
      return classes.moving;
    case RmfModels.DoorMode.MODE_CLOSED:
      return classes.close;
    default:
      return classes.unknown;
  }
}

function getDoorCenter(door: RmfModels.Door): [number, number] {
  const v1 = [door.v1_x, door.v1_y];
  const v2 = [door.v2_x, door.v2_y];
  switch (door.door_type) {
    case RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING:
    case RmfModels.Door.DOOR_TYPE_SINGLE_SWING:
    case RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE:
    case RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING:
    case RmfModels.Door.DOOR_TYPE_DOUBLE_SWING:
    case RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
      return [(v1[0] + v2[0]) / 2, (v2[1] + v1[1]) / 2];
    default:
      throw new Error('unknown door type');
  }
}

interface BaseDoorProps {
  /**
   * Start of the door in RMF coordinates
   */
  v1: [number, number];
  /**
   * End of the door in RMF coordinates
   */
  v2: [number, number];
  className?: string;
}

const BaseDoor = (props: BaseDoorProps) => {
  const { v1: v1_, v2: v2_, className } = props;
  const classes = useDoorStyles();

  const v1 = fromRmfCoords(v1_);
  const v2 = fromRmfCoords(v2_);

  return (
    <g>
      <line className={clsx(classes.base, className)} x1={v1[0]} y1={v1[1]} x2={v2[0]} y2={v2[1]} />
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
        className={clsx(classes.base, classes.transparent)}
        x1={v1[0]}
        y1={-v1[1]} // rmf y grows up while svg y grows down
        x2={v2[0]}
        y2={-v2[1]} // rmf y grows up while svg y grows down
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
  const { door, doorMode } = props;
  const doorStyle = useDoorStyle(doorMode);

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

/*
 * single/double telescoping doors:
 *   * common in elevators; same parameters as sliding doors; they just
 *     open/close faster and take up less space inside the wall.
 */
const SingleTelescopeDoor = SingleSlidingDoor;

/**
 * Double hinge doors:
 * - hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
 * - motion range = door swing ranges in DEGREES (assume symmetric)
 * - same motion-direction selection as single hinge
 */
const DoubleSwingDoor = (props: DoorMarkerImplProps) => {
  const { door, doorMode } = props;
  const [hingeX1, hingeY1, hingeX2, hingeY2] = [door.v1_x, door.v1_y, door.v2_x, door.v2_y];
  const [extendX1, extendY1] = [
    hingeX1 + (door.v2_x - door.v1_x) / 2,
    hingeY1 + (door.v2_y - door.v1_y) / 2,
  ];
  const doorStyle = useDoorStyle(doorMode);
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

/*
 * single/double telescoping doors:
 *   * common in elevators; same parameters as sliding doors; they just
 *     open/close faster and take up less space inside the wall.
 */
const DoubleTelescopeDoor = DoubleSlidingDoor;

/**
 * door: Door information provided by the map.
 * doorState: Current state of the door.
 * onClick: Action to trigger on click.
 */
export interface DoorMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  door: RmfModels.Door;
  doorMode?: number;
  /**
   * Whether the component should perform a translate transform to put it inline with the position
   * in RMF.
   *
   * default: true
   */
  translate?: boolean;
}

export const DoorMarker = React.forwardRef(
  (
    { door, doorMode, translate = true, ...otherProps }: DoorMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
    debug(`render ${door.name}`);
    const classes = useDoorStyles();

    const renderDoor = () => {
      switch (door.door_type) {
        case RmfModels.Door.DOOR_TYPE_SINGLE_SWING:
          return <SingleSwingDoor door={door} doorMode={doorMode} />;
        case RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING:
          return <SingleSlidingDoor door={door} doorMode={doorMode} />;
        case RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE:
          return <SingleTelescopeDoor door={door} doorMode={doorMode} />;
        case RmfModels.Door.DOOR_TYPE_DOUBLE_SWING:
          return <DoubleSwingDoor door={door} doorMode={doorMode} />;
        case RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING:
          return <DoubleSlidingDoor door={door} doorMode={doorMode} />;
        case RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
          return <DoubleTelescopeDoor door={door} doorMode={doorMode} />;
        default:
          return null;
      }
    };

    try {
      const center = getDoorCenter(door);
      return (
        <g ref={ref} {...otherProps}>
          <g
            className={otherProps.onClick ? classes.marker : undefined}
            transform={!translate ? `translate(${-center[0]} ${center[1]})` : undefined}
          >
            {renderDoor()}
          </g>
        </g>
      );
    } catch (e) {
      console.error((e as Error).message);
      return null;
    }
  },
);

export default DoorMarker;
