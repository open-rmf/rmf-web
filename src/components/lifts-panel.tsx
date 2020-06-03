import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import { LiftItem } from './lift-item';
import { SpotlightValue } from './spotlight-value';

interface LiftsPanelProps {
  lifts: readonly RomiCore.Lift[];
  liftStates: Readonly<Record<string, RomiCore.LiftState>>;
  transport?: Readonly<RomiCore.Transport>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onLiftRequest?(lift: RomiCore.Lift, destination: string): void;
  onLiftClick?(lift: RomiCore.Lift): void;
}

export default function LiftsPanel(props: LiftsPanelProps): JSX.Element {
  const { transport, spotlight, onLiftRequest, onLiftClick } = props;
  const liftRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.adapterLiftRequests) : null),
    [transport],
  );
  const liftRefs = React.useRef<Record<string, HTMLElement | null>>({});
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});

  function handleLiftRequest(lift: RomiCore.Lift, destination: string): void {
    liftRequestPub?.publish({
      lift_name: lift.name,
      door_state: RomiCore.LiftRequest.DOOR_OPEN,
      request_type: RomiCore.LiftRequest.REQUEST_AGV_MODE,
      request_time: RomiCore.toRosTime(new Date()),
      destination_floor: destination,
      session_id: transport!.name,
    });
    onLiftRequest && onLiftRequest(lift, destination);
  }

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = liftRefs.current[spotlight.value];
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
      {props.lifts.map(lift => {
        const liftState = props.liftStates[lift.name];
        return (
          <LiftItem
            key={lift.name}
            lift={lift}
            ref={ref => (liftRefs.current[lift.name] = ref)}
            liftState={liftState}
            enableRequest={Boolean(transport)}
            onRequest={handleLiftRequest}
            onClick={() => onLiftClick && onLiftClick(lift)}
            expanded={Boolean(expanded[lift.name])}
            onChange={(_, newExpanded) =>
              setExpanded(prev => ({
                ...prev,
                [lift.name]: newExpanded,
              }))
            }
          />
        );
      })}
    </React.Fragment>
  );
}
