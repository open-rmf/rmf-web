import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import Dispenser from './dispenser';
import { DispenserResource } from '../../resource-manager-dispensers';
import { ResourcesContext, DispenserStateContext } from '../app-contexts';

export interface DispensersOverlayProps extends SVGOverlayProps {
  onDispenserClick?(dispenser: RomiCore.DispenserState): void;
  currentFloorName: string;
}

export default function DispensersOverlay(props: DispensersOverlayProps): React.ReactElement {
  const { currentFloorName, onDispenserClick, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const footprint = 0.4;
  const dispenserResourcesContext = React.useContext(ResourcesContext).dispensers;
  const dispenserState = React.useContext(DispenserStateContext);
  /**
   * We choose to iterate the dispensers inside resources because we get the positions from the resources file and not from the dispenser's state. In case the dispenser doesn't have an entry in the resources file it will not appear in the map, but still will appear in the Omnipanel.
   */
  const dispenserInCurLevel = React.useMemo(() => {
    if (!dispenserResourcesContext) {
      return [];
    }
    return dispenserResourcesContext.allValues.filter(
      (d: DispenserResource) => d.location && d.location.level_name === currentFloorName,
    );
  }, [dispenserResourcesContext, currentFloorName]);

  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {dispenserInCurLevel.map((dispenser: Required<DispenserResource>) => {
          return (
            <Dispenser
              key={dispenser.guid}
              dispenser={dispenser}
              footprint={footprint}
              dispenserState={dispenserState[dispenser.guid]}
              onClick={(_, dispenser) => onDispenserClick && onDispenserClick(dispenser)}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
