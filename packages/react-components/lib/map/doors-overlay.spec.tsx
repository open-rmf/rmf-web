import { render } from '@testing-library/react';
import React from 'react';
import { DoorsOverlay } from './doors-overlay';
import { LMap } from './map';
import { officeL1Bounds, officeMap } from './test-utils.spec';

describe('DoorsOverlay', () => {
  it('smoke test', () => {
    const doors = officeMap.levels[0].doors;
    const root = render(
      <LMap bounds={officeL1Bounds}>
        <DoorsOverlay bounds={officeL1Bounds} doors={doors} />
      </LMap>,
    );
    expect(() => root.getByLabelText('main_door')).not.toThrow();
    expect(() => root.getByLabelText('coe_door')).not.toThrow();
    expect(() => root.getByLabelText('hardware_door')).not.toThrow();
  });
});
