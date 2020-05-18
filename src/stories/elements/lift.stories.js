import React from 'react';
import Lift from '../../components/schedule-visualizer/lift/lift';
import { liftStyles } from '../../components/schedule-visualizer/lift/liftContainer';

export default {
  title: 'Lift',
  component: Lift,
};

export const LiftState = () => {
  const classes = liftStyles();
  const liftBasic = {
    x: 0,
    y: 0,
    width: 100,
    height: 100,
    rx: 10,
    ry: 10,
  };
  const liftEmergency = { ...liftBasic, style: classes.emergency };
  const liftFire = { ...liftBasic, style: classes.fire };
  const liftOffLine = { ...liftBasic, style: classes.offLine };
  const liftHuman = { ...liftBasic, style: classes.humanMode };
  const liftUnknown = { ...liftBasic, style: classes.liftOnAnotherFloor };

  return (
    <>
      <h2>Emergency</h2>
      <svg width="100" height="100">
        <Lift
          lift={liftEmergency}
          liftMotionText={{
            x: 50,
            y: 50,
            text: 'L1->L2',
            fontSize: '0.8rem',
          }}
          liftModeText={{
            x: 50,
            y: 25,
            text: 'EMERGENCY!',
            fontSize: '0.8rem',
          }}
        />
      </svg>

      <h2>Fire</h2>
      <svg width="100" height="100">
        <Lift
          lift={liftFire}
          liftMotionText={{
            x: 50,
            y: 50,
            text: 'L1->L2',
          }}
          liftModeText={{
            x: 50,
            y: 25,
            text: 'FIRE!',
          }}
        />
      </svg>
      <h2>Offline</h2>
      <svg width="100" height="100">
        <Lift
          lift={liftOffLine}
          liftMotionText={{
            x: 50,
            y: 50,
            text: 'L1->L2',
          }}
          liftModeText={{
            x: 50,
            y: 25,
            text: 'OFFLINE!',
          }}
        />
      </svg>

      <h2>Human</h2>
      <svg width="100" height="100">
        <Lift
          lift={liftHuman}
          liftMotionText={{
            x: 50,
            y: 50,
            text: 'L1->L2',
          }}
          liftModeText={{
            x: 50,
            y: 25,
            text: 'HUMAN',
          }}
        />
      </svg>

      <h2>Unknown</h2>
      <svg width="100" height="100">
        <Lift
          lift={liftUnknown}
          liftMotionText={{
            x: 50,
            y: 50,
            text: undefined,
          }}
          liftModeText={{
            x: 50,
            y: 25,
            text: 'UNKNOWN',
          }}
        />
      </svg>
    </>
  );
};
