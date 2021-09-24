import { makeStyles } from '@material-ui/core';
import clsx from 'clsx';
import React, { SVGProps } from 'react';
import * as RmfModels from 'rmf-models';

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

const BaseDoor = ({ className, ...otherProps }: SVGProps<SVGLineElement>) => {
  const classes = useDoorStyles();
  return <line className={clsx(classes.base, className)} {...otherProps} />;
};

/**
 * Because we are using stroke-dash in some of the classes, it makes it such that only
 * the rendered line will be considered for click detection. To workaround it, we use
 * a transparent door on top of the marker, this dummy door will be used to allow the
 * full door to be clickable.
 */
const DummyDoor = ({ className, ...otherProps }: React.SVGProps<SVGLineElement>) => {
  const classes = useDoorStyles();
  return <line className={clsx(classes.base, classes.transparent, className)} {...otherProps} />;
};

type DoorMarkerImplProps = Omit<DoorMarkerProps, 'onClick'>;

const SingleSwingDoor = ({ x1, x2, y1, y2, doorMode }: DoorMarkerImplProps) => {
  const doorStyle = useDoorStyle(doorMode);

  return (
    <>
      <BaseDoor x1={x1} y1={y1} x2={x2} y2={y2} className={doorStyle} />
      <DummyDoor x1={x1} y1={y1} x2={x2} y2={y2} />
    </>
  );
};

const SingleSlidingDoor = SingleSwingDoor;

const SingleTelescopeDoor = SingleSlidingDoor;

const DoubleSwingDoor = ({ x1, y1, x2, y2, doorMode }: DoorMarkerImplProps) => {
  const separatorX = (x2 - x1) * 0.002;
  const separatorY = (y2 - y1) * 0.002;
  const centerX = x1 + (x2 - x1) / 2;
  const centerY = y1 + (y2 - y1) / 2;
  const doorStyle = useDoorStyle(doorMode);
  return (
    <>
      <BaseDoor
        x1={x1}
        y1={y1}
        x2={centerX - separatorX}
        y2={centerY - separatorY}
        className={doorStyle}
      />
      <BaseDoor
        x1={centerX + separatorX}
        y1={centerY + separatorY}
        x2={x2}
        y2={y2}
        className={doorStyle}
      />
      <DummyDoor x1={x1} y1={y1} x2={x2} y2={y2} />
    </>
  );
};

const DoubleSlidingDoor = DoubleSwingDoor;

const DoubleTelescopeDoor = DoubleSlidingDoor;

export interface DoorMarkerProps extends React.PropsWithRef<React.SVGProps<SVGGElement>> {
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  doorType: number;
  doorMode?: number;
}

export const DoorMarker = React.forwardRef(
  (props: DoorMarkerProps, ref: React.Ref<SVGGElement>) => {
    const { doorType, ...otherProps } = props;
    const classes = useDoorStyles();

    const renderDoor = () => {
      switch (doorType) {
        case RmfModels.Door.DOOR_TYPE_SINGLE_SWING:
          return <SingleSwingDoor {...props} />;
        case RmfModels.Door.DOOR_TYPE_SINGLE_SLIDING:
          return <SingleSlidingDoor {...props} />;
        case RmfModels.Door.DOOR_TYPE_SINGLE_TELESCOPE:
          return <SingleTelescopeDoor {...props} />;
        case RmfModels.Door.DOOR_TYPE_DOUBLE_SWING:
          return <DoubleSwingDoor {...props} />;
        case RmfModels.Door.DOOR_TYPE_DOUBLE_SLIDING:
          return <DoubleSlidingDoor {...props} />;
        case RmfModels.Door.DOOR_TYPE_DOUBLE_TELESCOPE:
          return <DoubleTelescopeDoor {...props} />;
        default:
          return null;
      }
    };

    try {
      return (
        <g ref={ref} {...otherProps}>
          <g className={otherProps.onClick ? classes.marker : undefined}>{renderDoor()}</g>
        </g>
      );
    } catch (e) {
      console.error((e as Error).message);
      return null;
    }
  },
);

export default DoorMarker;
