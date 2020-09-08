import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React, { useContext } from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { DoorStateContext } from '../rmf-contexts';
import Door, { DoorContainerProps } from './door/door';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:DoorsOverlay');

export interface DoorsOverlayProps extends SVGOverlayProps {
  doors: readonly RomiCore.Door[];
  onDoorClick?(door: RomiCore.Door): void;
}

export const DoorsOverlay = React.memo((props: DoorsOverlayProps) => {
  debug('render');

  const { doors, onDoorClick, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const doorsState = useContext(DoorStateContext);

  const getCurrentDoorMode = (doorName: string) => {
    const currentDoor = doorsState && doorsState[doorName];
    return currentDoor && currentDoor.current_mode.value;
  };

  const handleDoorClick = React.useCallback<Required<DoorContainerProps>['onClick']>(
    (_, door) => onDoorClick && onDoorClick(door),
    [onDoorClick],
  );

  return (
    <>
      <SVGOverlay {...otherProps}>
        <svg viewBox={viewBox}>
          {doors.map(door => (
            <Door
              key={`building-door-${door.name}`}
              door={door}
              onClick={handleDoorClick}
              doorState={doorsState && doorsState[door.name]}
              currentMode={getCurrentDoorMode(door.name)}
            />
          ))}
        </svg>
      </SVGOverlay>
    </>
  );
});

export default DoorsOverlay;
