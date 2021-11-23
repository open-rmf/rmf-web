import { render } from '@testing-library/react';
import type { Lift } from 'api-client';
import React from 'react';
import { LiftState as RmfLiftState } from 'rmf-models';
import { makeLift } from '../lifts/test-utils.spec';
import { getLiftModeVariant, LiftsOverlay } from './lifts-overlay';
import { LMap } from './map';
import { officeL1Bounds } from './test-utils.spec';

describe('LiftsOverlay', () => {
  it('smoke test', () => {
    const lifts: Lift[] = [makeLift({ name: 'test_lift' })];
    const root = render(
      <LMap bounds={officeL1Bounds}>
        <LiftsOverlay bounds={officeL1Bounds} currentLevel="L1" lifts={lifts} />
      </LMap>,
    );
    expect(() => root.getByLabelText('test_lift')).not.toThrow();
  });
});

describe('getLiftModeVariant picks style in correct order', () => {
  it('Returns unknown if liftStateMode and liftStateMode are undefined ', () => {
    expect(getLiftModeVariant('L1')).toBe('unknown');
  });

  it('picks emergency and fire mode if the lift is on the current floor and on a different floor', () => {
    const modeFire = getLiftModeVariant('L1', RmfLiftState.MODE_FIRE, 'L1');
    expect(modeFire).toBe('fire');

    const modeEmergency = getLiftModeVariant('L1', RmfLiftState.MODE_EMERGENCY, 'L2');
    expect(modeEmergency).toBe('emergency');
  });

  it('picks `offline` mode if the lift is on the current floor and a different floor', () => {
    const modeOfflineOnFloor = getLiftModeVariant('L1', RmfLiftState.MODE_OFFLINE, 'L1');
    expect(modeOfflineOnFloor).toBe('offLine');

    const modeOfflineOnAnotherFloor = getLiftModeVariant('L1', RmfLiftState.MODE_OFFLINE, 'L2');
    expect(modeOfflineOnAnotherFloor).toBe('offLine');
  });

  it('picks `moving` mode if the lift is on a different floor', () => {
    const modeMovingAgv = getLiftModeVariant('L1', RmfLiftState.MODE_AGV, 'L2');
    expect(modeMovingAgv).toBe('moving');
    const modeMovingHuman = getLiftModeVariant('L1', RmfLiftState.MODE_HUMAN, 'L2');
    expect(modeMovingHuman).toBe('moving');
  });

  it('picks AGV and HUMAN mode if the lift is on the same floor ', () => {
    const modeAgv = getLiftModeVariant('L1', RmfLiftState.MODE_AGV, 'L1');
    const modeHuman = getLiftModeVariant('L1', RmfLiftState.MODE_HUMAN, 'L1');
    expect(modeAgv).toBe('onCurrentLevel');
    expect(modeHuman).toBe('human');
  });
});
