import { makeStyles, useTheme } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const useStyles = makeStyles(() => ({
  placeMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));

export interface PlacesOverlayProps extends SVGOverlayProps {
  places: readonly RomiCore.Place[];
  onPlaceClick?(place: RomiCore.Place): void;
}

export default function PlacesOverlay(props: PlacesOverlayProps): React.ReactElement {
  const theme = useTheme();
  const classes = useStyles();
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const size = 1;

  return (
    <SVGOverlay {...props}>
      <svg viewBox={viewBox}>
        {props.places.map(place => (
          <g key={place.name}>
            <rect
              className={classes.placeMarker}
              x={place.x - size * 0.5}
              y={-place.y - size * 0.5}
              width={size}
              height={size}
              fill={theme.palette.primary.main}
              opacity={0.95}
              onClick={() => props.onPlaceClick && props.onPlaceClick(place)}
            />
          </g>
        ))}
      </svg>
    </SVGOverlay>
  );
}
