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

async function fontPathExists(url: string) {
  const result = await fetch(url, { method: 'HEAD' });
  return result.ok;
}

export const Lifts = React.memo(({ lift }: LiftsProps): JSX.Element => {
  const rmf = React.useContext(RmfAppContext);
  const [liftState, setLiftState] = React.useState<LiftState | undefined>(undefined);
  const [fontPath, setFontPath] = React.useState<string | undefined>(undefined);

  React.useEffect(() => {
    const loadFont = async () => {
      try {
        const newFontPath =
          process.env.PUBLIC_URL && process.env.PUBLIC_URL.length > 0
            ? `${process.env.PUBLIC_URL}/roboto-v18-KFOmCnqEu92Fr1Mu4mxM.woff`
            : '/roboto-v18-KFOmCnqEu92Fr1Mu4mxM.woff';

        if (await fontPathExists(newFontPath)) {
          setFontPath(newFontPath);
        }
      } catch (error) {
        console.error('Error loading font:', error);
      }
    };

    loadFont();
  }, []);

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
          fontPath={fontPath}
        />
      )}
    </>
  );
});
