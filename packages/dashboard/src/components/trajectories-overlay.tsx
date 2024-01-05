import React from 'react';
import {
  SVGOverlay,
  SVGOverlayProps,
  TrajectoryMarker as TrajectoryMarker_,
  TrajectoryMarkerProps,
  viewBoxFromLeafletBounds,
} from 'react-components';

const TrajectoryMarker = React.memo(TrajectoryMarker_);

export type TrajectoryData = TrajectoryMarkerProps;

export interface TrajectoriesOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  trajectoriesData: TrajectoryData[];
}

export const TrajectoriesOverlay = React.memo(
  ({ trajectoriesData, ...otherProps }: TrajectoriesOverlayProps): JSX.Element => {
    const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);

    return (
      <SVGOverlay viewBox={viewBox} {...otherProps}>
        {trajectoriesData.map((trajData) => (
          <TrajectoryMarker
            key={trajData.trajectory.id}
            aria-label={`trajectory ${trajData.trajectory.id}`}
            {...trajData}
          />
        ))}
      </SVGOverlay>
    );
  },
);
