import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import ColorManager from './colors';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { ResourcesContext } from '../../app-contexts';
import Dispenser from './dispenser';
import { ResourceDispenserConfigurationInterface } from '../../resource-manager-dispensers';

export interface DispensersOverlayProps extends SVGOverlayProps {
  colorManager: ColorManager;
  onDispenserClick?(): void;
  currentFloorName: string;
}

export default function DispensersOverlay(props: DispensersOverlayProps): React.ReactElement {
  const { colorManager, currentFloorName, onDispenserClick, ...otherProps } = props;
  const viewBox = viewBoxFromLeafletBounds(props.bounds);
  const footprint = 0.5;
  const resourcesContext = React.useContext(ResourcesContext);
  const dispenserInCurLevel = React.useMemo(() => {
    return resourcesContext.dispensers.allValues.filter(
      (d: ResourceDispenserConfigurationInterface) =>
        d.location && d.location.level_name === currentFloorName,
    );
  }, []);
  return (
    <SVGOverlay {...otherProps}>
      <svg viewBox={viewBox}>
        {dispenserInCurLevel.map((dispenser: ResourceDispenserConfigurationInterface) => {
          return (
            <Dispenser
              key={dispenser.name}
              dispenser={dispenser}
              footprint={footprint}
              colorManager={colorManager}
              onClick={() => onDispenserClick && onDispenserClick()}
            />
          );
        })}
      </svg>
    </SVGOverlay>
  );
}
