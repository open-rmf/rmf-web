import { makeStyles, useTheme } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export interface PlaceProps {
  place: RomiCore.Place;
  size: number;
  onClick?(e: React.MouseEvent<SVGGElement>, place: RomiCore.Place): void;
}

const Place = React.forwardRef(function(
  props: PlaceProps,
  ref: React.Ref<SVGGElement>,
): React.ReactElement {
  const { place, size, onClick } = props;
  const classes = useStyles();
  const theme = useTheme();
  return (
    <g ref={ref} onClick={e => onClick && onClick(e as any, place)}>
      <rect
        className={classes.placeMarker}
        x={place.x - size * 0.5}
        y={-place.y - size * 0.5}
        width={size}
        height={size}
        fill={theme.palette.primary.main}
        opacity={0.95}
      />
    </g>
  );
});

export default Place;

const useStyles = makeStyles(() => ({
  placeMarker: {
    cursor: 'pointer',
    pointerEvents: 'auto',
  },
}));
