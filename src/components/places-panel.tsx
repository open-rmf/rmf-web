import { Divider, List, ListItem, ListItemText } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { SpotlightValue } from './spotlight-expansion-panel';

export interface PlacesPanelProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onPlaceClick?(place: RomiCore.Place): void;
}

export default function PlacesPanel(props: PlacesPanelProps): React.ReactElement {
  const spotlightRef = React.useRef<HTMLElement>(null);

  React.useEffect(() => {
    spotlightRef.current?.scrollIntoView();
  }, [props.spotlight]);

  function handlePlaceClick(place: RomiCore.Place): void {
    props.onPlaceClick && props.onPlaceClick(place);
  }

  return (
    <List>
      {props.buildingMap.levels.map(level =>
        level.places.map(place => (
          <React.Fragment key={place.name}>
            <ListItem button={true} onClick={() => handlePlaceClick(place)}>
              <ListItemText
                ref={
                  props.spotlight && props.spotlight.value === place.name ? spotlightRef : undefined
                }
                primary={place.name}
                primaryTypographyProps={{ variant: 'h5' }}
                secondary={level.name}
              />
            </ListItem>
            <Divider />
          </React.Fragment>
        )),
      )}
    </List>
  );
}
