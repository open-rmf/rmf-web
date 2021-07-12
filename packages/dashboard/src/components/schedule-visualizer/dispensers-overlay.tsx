import Debug from 'debug';
import React from 'react';
import { DispenserMarker as DispenserMarker_, DispenserMarkerProps } from 'react-components';
import { DispenserResource } from '../../managers/resource-manager-dispensers';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { ResourcesContext } from '../app-contexts';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:DispensersOverlay');
const DispenserMarker = React.memo(DispenserMarker_);

export interface DispensersOverlayProps extends SVGOverlayProps {
  currentFloorName: string;
  onDispenserClick?(event: React.MouseEvent, guid: string): void;
  MarkerComponent?: React.ComponentType<DispenserMarkerProps>;
}

export const DispensersOverlay = (props: DispensersOverlayProps): React.ReactElement => {
  debug('render');

  const {
    currentFloorName,
    onDispenserClick,
    MarkerComponent = DispenserMarker,
    ...otherProps
  } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const footprint = 0.4;
  const dispenserResourcesContext = React.useContext(ResourcesContext)?.dispensers;

  /**
   * We choose to iterate the dispensers inside resources because we get the positions
   * from the resources file and not from the dispenser's state. In case the dispenser
   * doesn't have an entry in the resources file it will not appear in the map, but
   * still will appear in the Omnipanel.
   */
  const dispenserInCurLevel = React.useMemo(() => {
    if (!dispenserResourcesContext) {
      return [];
    }
    return dispenserResourcesContext.allValues.filter(
      (d: DispenserResource) => d.location && d.location.level_name === currentFloorName,
    );
  }, [dispenserResourcesContext, currentFloorName]);

  const dispenserLocations = React.useMemo(
    () =>
      dispenserInCurLevel.map(
        (dispenser) => [dispenser.location.x, dispenser.location.y] as [number, number],
      ),
    [dispenserInCurLevel],
  );

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {dispenserResourcesContext &&
          dispenserInCurLevel.map((dispenser, idx) => {
            return (
              <MarkerComponent
                key={dispenser.guid}
                guid={dispenser.guid}
                location={dispenserLocations[idx]}
                iconPath={dispenserResourcesContext.getIconPath(dispenser.guid) || undefined}
                footprint={footprint}
                onClick={onDispenserClick}
                aria-label={dispenser.guid}
                data-component="DispenserMarker"
                data-testid="dispenserMarker"
              />
            );
          })}
      </svg>
    </SVGOverlay>
  );
};

export default DispensersOverlay;
