import React from 'react';
import {
  TrajectoryMarker as TrajectoryMarker_,
  TrajectoryMarkerProps,
  viewBoxFromLeafletBounds,
} from 'react-components';
import { SVGOverlay, SVGOverlayProps } from 'react-leaflet';

const TrajectoryMarker = React.memo(TrajectoryMarker_);

export type TrajectoryData = TrajectoryMarkerProps;

export interface TrajectoriesOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  trajectoriesData: TrajectoryData[];
}

export const TrajectoriesOverlay = React.memo(
  ({ trajectoriesData, ...otherProps }: TrajectoriesOverlayProps): JSX.Element => {
    const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);

    return (
      <SVGOverlay attributes={{ viewBox: viewBox }} {...otherProps}>
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
