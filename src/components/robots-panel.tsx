import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import RobotItem, { RobotItemProps } from './robot-item';
import { SpotlightValue } from './spotlight-value';

export interface RobotsPanelProps {
  fleets: readonly RomiCore.FleetState[];
  spotlight?: Readonly<SpotlightValue<string>>;
}

export const RobotsPanel = React.memo((props: RobotsPanelProps) => {
  const { fleets, spotlight } = props;
  const robotRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});

  const storeRef = React.useCallback((ref: HTMLElement | null) => {
    if (!ref) {
      return;
    }
    const guid = ref.getAttribute('data-guid');
    if (!guid) {
      return;
    }
    robotRefs.current[guid] = ref;
  }, []);

  const onChange = React.useCallback<Required<RobotItemProps>['onChange']>((event, newExpanded) => {
    const fleetName = (event.currentTarget as HTMLElement).parentElement?.getAttribute(
      'data-fleet',
    );
    const robotName = (event.currentTarget as HTMLElement).parentElement?.getAttribute('data-name');
    if (!fleetName || !robotName) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [`${fleetName}-${robotName}`]: newExpanded,
    }));
  }, []);

  const transitionProps = React.useMemo(() => ({ unmountOnExit: true }), []);

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = robotRefs.current[spotlight.value];
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
      {fleets.flatMap(fleet =>
        fleet.robots.map(robot => (
          <RobotItem
            key={`${fleet.name}-${robot.name}`}
            data-fleet={fleet.name}
            data-name={robot.name}
            ref={storeRef}
            fleetName={fleet.name}
            robot={robot}
            expanded={Boolean(expanded[`${fleet.name}-${robot.name}`])}
            onChange={onChange}
            TransitionProps={transitionProps}
          />
        )),
      )}
    </React.Fragment>
  );
});

export default RobotsPanel;
