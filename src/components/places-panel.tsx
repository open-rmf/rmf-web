import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { PlaceItem } from './place-item';
import { SpotlightValue } from './spotlight-value';

export interface PlacesPanelProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onPlaceClick?(place: RomiCore.Place): void;
}

export default function PlacesPanel(props: PlacesPanelProps): React.ReactElement {
  const { buildingMap, spotlight, onPlaceClick } = props;
  const placeRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = placeRefs.current[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight]);

  return (
    <React.Fragment>
      {buildingMap.levels.flatMap(level =>
        level.places.map(place => (
          <PlaceItem
            key={place.name}
            ref={ref => (placeRefs.current[place.name] = ref)}
            place={place}
            level={level}
            onClick={() => onPlaceClick && onPlaceClick(place)}
            expanded={Boolean(expanded[place.name])}
            onChange={(_, newExpanded) =>
              setExpanded(prev => ({
                ...prev,
                [place.name]: newExpanded,
              }))
            }
          />
        )),
      )}
    </React.Fragment>
  );
}
