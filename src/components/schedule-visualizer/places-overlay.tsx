import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import Place from './place';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

export interface PlacesOverlayProps extends SVGOverlayProps {
  places: any; //readonly RomiCore.Place[];
  onPlaceClick?(place: RomiCore.Place): void;
}

export default function PlacesOverlay(props: PlacesOverlayProps): React.ReactElement {
  const { places, onPlaceClick, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const size = 1;

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {props.places.map((place: any) => (
          <Place
            key={place.name}
            place={place}
            size={size}
            onClick={(_, place_) => onPlaceClick && onPlaceClick(place_)}
          />
        ))}
      </svg>
    </SVGOverlay>
  );
}
