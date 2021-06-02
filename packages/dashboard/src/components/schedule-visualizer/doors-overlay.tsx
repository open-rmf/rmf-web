import Debug from 'debug';
import React, { useContext } from 'react';
import { DoorMarker as DoorMarker_, DoorMarkerProps } from 'react-components';
import * as RmfModels from 'rmf-models';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { DoorStateContext } from '../rmf-app';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:DoorsOverlay');
const DoorMarker = React.memo(DoorMarker_);

export interface DoorsOverlayProps extends SVGOverlayProps {
  doors: RmfModels.Door[];
  onDoorClick?(door: RmfModels.Door): void;
}

export const DoorsOverlay = (props: DoorsOverlayProps) => {
  debug('render');

  const { doors, onDoorClick, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const doorsState = useContext(DoorStateContext);

  const v1s = React.useMemo(() => doors.map((door) => [door.v1_x, door.v1_y] as [number, number]), [
    doors,
  ]);
  const v2s = React.useMemo(() => doors.map((door) => [door.v2_x, door.v2_y] as [number, number]), [
    doors,
  ]);

  const handleDoorClick = React.useCallback<Required<DoorMarkerProps>['onClick']>(
    (ev) => {
      const dataIdx = ev.currentTarget.getAttribute('data-index');
      if (dataIdx === null) {
        console.error('error handling door marker click (data-index not found)');
        return;
      }
      const idx = parseInt(dataIdx);
      if (isNaN(idx) || idx > doors.length - 1) {
        console.error('error handling door marker click (data-index is not a valid index');
        return;
      }
      onDoorClick && onDoorClick(doors[idx]);
    },
    [doors, onDoorClick],
  );

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {doors.map((door, idx) => (
          <DoorMarker
            key={door.name}
            onClick={handleDoorClick}
            v1={v1s[idx]}
            v2={v2s[idx]}
            doorType={door.door_type}
            doorMode={
              doorsState && doorsState[door.name] && doorsState[door.name].current_mode.value
            }
            aria-label={door.name}
            data-component="DoorMarker"
            data-testid="doorMarker"
            data-index={idx}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default DoorsOverlay;
