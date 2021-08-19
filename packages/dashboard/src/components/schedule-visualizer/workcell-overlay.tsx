import Debug from 'debug';
import React from 'react';
import { WorkcellMarker as WorkcellMarker_, WorkcellMarkerProps, useAsync } from 'react-components';
import { DispenserResource } from '../../managers/resource-manager-dispensers';
import { viewBoxFromLeafletBounds } from '../../util/css-utils';
import { ResourcesContext } from '../app-contexts';
import SVGOverlay, { SVGOverlayProps } from './svg-overlay';

const debug = Debug('ScheduleVisualizer:DispensersOverlay');
const DispenserMarker = React.memo(WorkcellMarker_);

export interface WorkcellsOverlayProps extends SVGOverlayProps {
  currentFloorName: string;
  onDispenserClick?(event: React.MouseEvent, guid: string): void;
  MarkerComponent?: React.ComponentType<WorkcellMarkerProps>;
}

export const WorkcellsOverlay = (props: WorkcellsOverlayProps): React.ReactElement => {
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
  const safeAsync = useAsync();
  const [iconPaths, setIconPaths] = React.useState<Record<string, string>>({});

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

  React.useEffect(() => {
    if (!dispenserResourcesContext) return;
    (async () => {
      const newIcons: Record<string, string> = {};
      const ps = dispenserInCurLevel.map(async (dispenser) => {
        const icon = await dispenserResourcesContext.getIconPath(dispenser.guid);
        if (icon) newIcons[dispenser.guid] = icon;
      });
      await safeAsync(Promise.all(ps));
      setIconPaths((prev) => ({ ...prev, ...newIcons }));
    })();
  }, [dispenserResourcesContext, dispenserInCurLevel, safeAsync]);

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
                iconPath={iconPaths[dispenser.guid]}
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

export default WorkcellsOverlay;
