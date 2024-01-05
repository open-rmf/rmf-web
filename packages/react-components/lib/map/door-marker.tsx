import { styled } from '@mui/material';
import clsx from 'clsx';
import React, { SVGProps } from 'react';
import { Door as RmfDoor, DoorMode as RmfDoorMode } from 'rmf-models';

const classes = {
  marker: 'door-marker-base-marker',
  base: 'door-marker-base-door',
  open: 'door-marker-open',
  close: 'door-marker-close',
  moving: 'door-marker-moving',
  unknown: 'door-marker-unknown',
  transparent: 'door-marker-transparent',
};
const StyledG = styled('g')(() => ({
  [`& .${classes.marker}`]: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
  [`& .${classes.base}`]: {
    strokeWidth: 0.2,
  },
  [`& .${classes.open}`]: {
    width: 200,
    stroke: '#AFDDAE',
    strokeDasharray: 0.1,
  },
  [`& .${classes.close}`]: {
    stroke: '#BC4812',
  },
  [`& .${classes.moving}`]: {
    stroke: '#E9CE9F',
    strokeDasharray: 0.3,
  },
  [`& .${classes.unknown}`]: {
    stroke: 'grey',
  },
  [`& .${classes.transparent}`]: {
    stroke: 'transparent',
  },
}));

function useDoorStyle(doorMode?: number): string {
  if (doorMode === undefined) {
    return classes.unknown;
  }

  switch (doorMode) {
    case RmfDoorMode.MODE_OPEN:
      return classes.open;
    case RmfDoorMode.MODE_MOVING:
      return classes.moving;
    case RmfDoorMode.MODE_CLOSED:
      return classes.close;
    default:
      return classes.unknown;
  }
}

const BaseDoor = ({ className, ...otherProps }: SVGProps<SVGLineElement>) => {
  return <line className={clsx(classes.base, className)} {...otherProps} />;
};

/**
 * Because we are using stroke-dash in some of the classes, it makes it such that only
 * the rendered line will be considered for click detection. To workaround it, we use
 * a transparent door on top of the marker, this dummy door will be used to allow the
 * full door to be clickable.
 */
const DummyDoor = ({ className, ...otherProps }: React.SVGProps<SVGLineElement>) => {
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
  (
    { x1, y1, x2, y2, doorType, doorMode, ...otherProps }: DoorMarkerProps,
    ref: React.Ref<SVGGElement>,
  ) => {
    const doorProps = { x1, y1, x2, y2, doorType, doorMode };

    const renderDoor = () => {
      switch (doorType) {
        case RmfDoor.DOOR_TYPE_SINGLE_SWING:
          return <SingleSwingDoor {...doorProps} />;
        case RmfDoor.DOOR_TYPE_SINGLE_SLIDING:
          return <SingleSlidingDoor {...doorProps} />;
        case RmfDoor.DOOR_TYPE_SINGLE_TELESCOPE:
          return <SingleTelescopeDoor {...doorProps} />;
        case RmfDoor.DOOR_TYPE_DOUBLE_SWING:
          return <DoubleSwingDoor {...doorProps} />;
        case RmfDoor.DOOR_TYPE_DOUBLE_SLIDING:
          return <DoubleSlidingDoor {...doorProps} />;
        case RmfDoor.DOOR_TYPE_DOUBLE_TELESCOPE:
          return <DoubleTelescopeDoor {...doorProps} />;
        default:
          return null;
      }
    };

    try {
      return (
        <StyledG ref={ref} {...otherProps}>
          <g className={otherProps.onClick ? classes.marker : undefined}>{renderDoor()}</g>
        </StyledG>
      );
    } catch (e) {
      console.error((e as Error).message);
      return null;
    }
  },
);

export default DoorMarker;
