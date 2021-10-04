import React from 'react';
import { SVGOverlay, SVGOverlayProps } from 'react-leaflet';
import { TrajectoryMarker as TrajectoryMarker_, TrajectoryMarkerProps } from './trajectory-marker';
import { viewBoxFromLeafletBounds } from './utils';

const TrajectoryMarker = React.memo(TrajectoryMarker_);

export type TrajectoryData = TrajectoryMarkerProps;

export interface TrajectoriesOverlayProps extends SVGOverlayProps {
  trajectoriesData: TrajectoryData[];
}

export const TrajectoriesOverlay = ({
  trajectoriesData,
  bounds,
  ...otherProps
}: TrajectoriesOverlayProps): JSX.Element => {
  const viewBox = viewBoxFromLeafletBounds(bounds);

  return (
    <SVGOverlay bounds={bounds} {...otherProps}>
      <svg viewBox={viewBox}>
        {trajectoriesData.map((trajData) => (
          <TrajectoryMarker
            key={trajData.trajectory.id}
            aria-label={`trajectory ${trajData.trajectory.id}`}
            {...trajData}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
};

export default TrajectoriesOverlay;
