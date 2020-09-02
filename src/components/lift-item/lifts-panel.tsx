import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { LiftRequest } from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import React from 'react';
import { makeCallbackArrayCallback } from '../../util/react-helpers';
import { SpotlightValue } from '../spotlight-value';
import { LiftItem, LiftItemProps } from './lift-item';

const debug = Debug('LiftsPanel');

export interface LiftsPanelProps {
  lifts: RomiCore.Lift[];
  liftStates: Readonly<Record<string, RomiCore.LiftState>>;
  transport?: Readonly<RomiCore.Transport>;
  spotlight?: Readonly<SpotlightValue<string>>;
  onLiftRequest?(lift: RomiCore.Lift, destination: string): void;
  onLiftClick?(lift: RomiCore.Lift): void;
}

function handleLiftRequest(
  liftRequestPub: RomiCore.Publisher<LiftRequest>,
  sessionId: string,
  lift: RomiCore.Lift,
  doorState: number,
  requestType: number,
  destination: string,
): void {
  liftRequestPub.publish({
    lift_name: lift.name,
    door_state: doorState,
    request_type: requestType,
    request_time: RomiCore.toRosTime(new Date()),
    destination_floor: destination,
    session_id: sessionId,
  });
}

export const LiftsPanel = React.memo((props: LiftsPanelProps) => {
  debug('render');

  const { lifts, liftStates, transport, spotlight, onLiftRequest, onLiftClick } = props;
  const liftRequestPub = React.useMemo(
    () => (transport ? transport.createPublisher(RomiCore.adapterLiftRequests) : null),
    [transport],
  );
  const [expanded, setExpanded] = React.useState<Readonly<Record<string, boolean>>>({});

  const liftRefs = React.useMemo(() => {
    const refs: Record<string, React.RefObject<HTMLElement>> = {};
    lifts.map(lift => (refs[lift.name] = React.createRef()));
    return refs;
  }, [lifts]);

  React.useEffect(() => {
    if (!spotlight) {
      return;
    }
    const ref = liftRefs[spotlight.value];
    if (!ref) {
      return;
    }
    setExpanded(prev => ({
      ...prev,
      [spotlight.value]: true,
    }));
    ref.current?.scrollIntoView({ behavior: 'smooth' });
  }, [spotlight, liftRefs]);

  const onChange = React.useMemo(
    makeCallbackArrayCallback<Required<LiftItemProps>['onChange'], RomiCore.Lift>(
      lifts,
      (lift, _, newExpanded) =>
        setExpanded(prev => ({
          ...prev,
          [lift.name]: newExpanded,
        })),
    ),
    [lifts],
  );

  const onClick = React.useMemo(
    makeCallbackArrayCallback<Required<LiftItemProps>['onClick'], RomiCore.Lift>(
      lifts,
      lift => onLiftClick && onLiftClick(lift),
    ),
    [lifts, onLiftClick],
  );

  const onRequest = React.useMemo(
    makeCallbackArrayCallback<Required<LiftItemProps>['onRequest'], RomiCore.Lift>(
      lifts,
      (lift, _, doorState, requestType, destination) => {
        liftRequestPub &&
          transport &&
          handleLiftRequest(
            liftRequestPub,
            transport.name,
            lift,
            doorState,
            requestType,
            destination,
          );
        onLiftRequest && onLiftRequest(lift, destination);
      },
    ),
    [],
  );

  return (
    <React.Fragment>
      {lifts.map((lift, i) => {
        const liftState = liftStates[lift.name];
        return (
          <LiftItem
            key={lift.name}
            id={`LiftItem-${lift.name}`}
            lift={lift}
            ref={liftRefs[lift.name]}
            liftState={liftState}
            enableRequest={Boolean(transport)}
            onRequest={onRequest[i]}
            onClick={onClick[i]}
            expanded={Boolean(expanded[lift.name])}
            onChange={onChange[i]}
          />
        );
      })}
    </React.Fragment>
  );
});

export default LiftsPanel;
