import React from 'react';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { WorkcellMarker as WorkcellMarker_, WorkcellMarkerProps } from './workcell-marker';

const WorkcellMarker = React.memo(WorkcellMarker_);

export interface WorkcellData {
  guid: string;
  location: [x: number, y: number];
  iconPath?: string;
}

export interface WorkcellsOverlayProps extends SVGOverlayProps {
  workcells: WorkcellData[];
  onWorkcellClick?(event: React.MouseEvent, guid: string): void;
  MarkerComponent?: React.ComponentType<WorkcellMarkerProps>;
}

export const WorkcellsOverlay = ({
  workcells,
  onWorkcellClick,
  MarkerComponent = WorkcellMarker,
  bounds,
  ...otherProps
}: WorkcellsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);
  const footprint = 0.4;

  const onClickHandlers = React.useMemo(
    () =>
      workcells.map<React.MouseEventHandler>((wc) => (ev) =>
        onWorkcellClick && onWorkcellClick(ev, wc.guid),
      ),
    [workcells, onWorkcellClick],
  );

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {workcells.map((workcell, idx) => {
          return (
            <MarkerComponent
              key={workcell.guid}
              guid={workcell.guid}
              location={workcell.location}
              footprint={footprint}
              iconPath={workcell.iconPath}
              onClick={onClickHandlers[idx]}
              aria-label={workcell.guid}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
};

export default WorkcellsOverlay;
