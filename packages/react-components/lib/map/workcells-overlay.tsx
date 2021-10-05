import React from 'react';
import { fromRmfCoords } from '../utils/geometry';
import { useAutoScale } from './hooks';
import { SVGOverlay, SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { withLabel } from './with-label';
import { WorkcellMarker as WorkcellMarker_, WorkcellMarkerProps } from './workcell-marker';

interface BoundedMarkerProps extends Omit<WorkcellMarkerProps, 'onClick'> {
  guid: string;
  onClick?: (ev: React.MouseEvent, guid: string) => void;
}

/**
 * Bind a marker to include the guid in the click event.
 * This is needed to avoid re-rendering all markers when only one of them changes.
 */
function bindMarker(MarkerComponent: React.ComponentType<WorkcellMarkerProps>) {
  return ({ guid, onClick, ...otherProps }: BoundedMarkerProps) => {
    const handleClick = React.useCallback((ev) => onClick && onClick(ev, guid), [onClick, guid]);
    return <MarkerComponent onClick={onClick && handleClick} {...otherProps} />;
  };
}

const WorkcellMarker = withLabel(bindMarker(WorkcellMarker_));

export interface WorkcellData {
  guid: string;
  location: [x: number, y: number];
  iconPath?: string;
}

export interface WorkcellsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  workcells: WorkcellData[];
  actualSizeMinZoom?: number;
  hideLabels?: boolean;
  onWorkcellClick?: (event: React.MouseEvent, guid: string) => void;
}

export const WorkcellsOverlay = ({
  workcells,
  hideLabels = false,
  onWorkcellClick,
  ...otherProps
}: WorkcellsOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
  const scale = useAutoScale(40);

  return (
    <SVGOverlay viewBox={viewBox} {...otherProps}>
      {workcells.map((workcell) => {
        const [x, y] = fromRmfCoords(workcell.location);
        return (
          <g key={workcell.guid}>
            <WorkcellMarker
              cx={x}
              cy={y}
              size={1}
              guid={workcell.guid}
              iconPath={workcell.iconPath}
              onClick={onWorkcellClick}
              aria-label={workcell.guid}
              style={{ transform: `scale(${scale})`, transformOrigin: `${x}px ${y}px` }}
              labelText={workcell.guid}
              labelSourceX={x}
              labelSourceY={y}
              labelSourceRadius={0.5 * scale}
              hideLabel={hideLabels}
            />
          </g>
        );
      })}
    </SVGOverlay>
  );
};

export default WorkcellsOverlay;
