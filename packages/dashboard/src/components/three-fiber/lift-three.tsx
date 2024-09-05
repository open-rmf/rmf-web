import robotoFont from '@fontsource/roboto/files/roboto-latin-400-normal.woff';
import { ThreeEvent } from '@react-three/fiber';
import { Lift, LiftState } from 'api-client';
import React from 'react';
import { LiftThreeMaker } from 'react-components';
import { throttleTime } from 'rxjs';

import { useRmfApi } from '../../hooks/use-rmf-api';

interface LiftsProps {
  opacity: number;
  height: number;
  elevation: number;
  lift?: Lift;
  onLiftClick?: (ev: ThreeEvent<MouseEvent>, lift: Lift) => void;
}

export const Lifts = React.memo(({ lift, onLiftClick }: LiftsProps): JSX.Element => {
  const rmfApi = useRmfApi();
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);

  React.useEffect(() => {
    if (!lift) {
      return;
    }

    const sub = rmfApi
      .getLiftStateObs(lift.name)
      .pipe(throttleTime(3000, undefined, { leading: true, trailing: true }))
      .subscribe(setLiftState);
    return () => sub.unsubscribe();
  }, [rmfApi, lift]);

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
          fontPath={robotoFont}
          onLiftClick={(ev: ThreeEvent<MouseEvent>) => onLiftClick && onLiftClick(ev, lift)}
        />
      )}
    </>
  );
});
