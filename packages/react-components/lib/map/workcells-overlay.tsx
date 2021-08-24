import React from 'react';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { WorkcellMarker as WorkcellMarker_, WorkcellMarkerProps } from './workcell-marker';

const WorkcellMarker = React.memo(WorkcellMarker_);

interface BoundedMarkerProps extends Omit<WorkcellMarkerProps, 'onClick'> {
  onClick?: (ev: React.MouseEvent, guid: string) => void;
}

/**
 * Bind a marker to include the guid in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<WorkcellMarkerProps>) {
  return ({ onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback((ev) => onClick && onClick(ev, otherProps.guid), [
      onClick,
      otherProps.guid,
    ]);
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

export interface WorkcellData {
  guid: string;
  location: [x: number, y: number];
  iconPath?: string;
}

export interface WorkcellsOverlayProps extends SVGOverlayProps {
  workcells: WorkcellData[];
  onWorkcellClick?: (event: React.MouseEvent, guid: string) => void;
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

  const BoundedMarker = React.useMemo(() => bindMarker(MarkerComponent), [MarkerComponent]);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {workcells.map((workcell) => {
          return (
            <BoundedMarker
              key={workcell.guid}
              guid={workcell.guid}
              location={workcell.location}
              footprint={footprint}
              iconPath={workcell.iconPath}
              onClick={onWorkcellClick}
              aria-label={workcell.guid}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
};

export default WorkcellsOverlay;
