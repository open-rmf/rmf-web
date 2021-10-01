import React from 'react';
import { fromRmfCoords } from '../utils/geometry';
import { useAutoScale } from './hooks';
import { ScaledNameLabel } from './label-marker';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from './utils';
import { WorkcellMarker as WorkcellMarker_, WorkcellMarkerProps } from './workcell-marker';

const WorkcellMarker = React.memo(WorkcellMarker_);

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

export interface WorkcellData {
  guid: string;
  location: [x: number, y: number];
  iconPath?: string;
}

export interface WorkcellsOverlayProps extends SVGOverlayProps {
  workcells: WorkcellData[];
  actualSizeMinZoom?: number;
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
  const scale = useAutoScale(40);

  const BoundedMarker = React.useMemo(() => bindMarker(MarkerComponent), [MarkerComponent]);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {workcells.map((workcell) => {
          const [x, y] = fromRmfCoords(workcell.location);
          return (
            <g key={workcell.guid}>
              <BoundedMarker
                cx={x}
                cy={y}
                size={1}
                guid={workcell.guid}
                iconPath={workcell.iconPath}
                onClick={onWorkcellClick}
                aria-label={workcell.guid}
                style={{ transform: `scale(${scale})`, transformOrigin: `${x}px ${y}px` }}
              />
              <ScaledNameLabel
                text={workcell.guid}
                sourceX={x}
                sourceY={y}
                sourceRadius={0.5 * scale}
              />
            </g>
          );
        })}
      </svg>
    </SVGOverlay>
  );
};

export default WorkcellsOverlay;
