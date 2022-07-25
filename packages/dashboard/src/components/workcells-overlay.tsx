import React from 'react';
import {
  fromRmfCoords,
  SVGOverlay,
  SVGOverlayProps,
  useAutoScale,
  viewBoxFromLeafletBounds,
  withLabel,
  WorkcellMarker as BaseWorkcellMarker,
} from 'react-components';

const WorkcellMarker = withLabel(BaseWorkcellMarker);

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

export const WorkcellsOverlay = React.memo(
  ({
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
                iconPath={workcell.iconPath}
                onClick={(ev) => onWorkcellClick && onWorkcellClick(ev, workcell.guid)}
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
  },
);
