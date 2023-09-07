import React from 'react';
import { Lift, LiftState } from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { LiftThreeMaker } from 'react-components';

interface LiftsProps {
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
}

export const Lifts = React.memo(({ lift }: LiftsProps): JSX.Element => {
  const rmf = React.useContext(RmfAppContext);
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    if (!lift) {
      return;
    }

    const sub = rmf.getLiftStateObs(lift.name).subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmf, lift]);

  return (
    <>
      {lift && liftState && (
        <LiftThreeMaker
          x={lift.ref_x}
          y={lift.ref_y}
          yaw={lift.ref_yaw}
          width={lift.width}
          depth={lift.depth}
          liftState={liftState}
        />
      )}
    </>
  );
});
